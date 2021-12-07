package ShapeAndPoseModels

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, PointId}
import scalismo.common.interpolation.{FieldInterpolator, NearestNeighborInterpolator3D}
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.numerics.PivotedCholesky
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess.Eigenpair
import scalismo.statisticalmodel.{DiscreteLowRankGaussianProcess, PointDistributionModel}
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.transformations.RigidTransformation
/**
  * A ShapeAndPoseGPA models shape and pose variations across given examples of mesh with Pose Parameters as deformation fields using [[DiscreteLowRankGaussianProcess]].
  * the ShapeAndPoseGPA while modelling shape deformation fields as [[scalismo.statisticalmodel.StatisticalMeshModel]], it uses [[PoseExpLogMapping]] to linearise pose deformation fields.
  * ShapeAndPoseGPA warps the single body object with the shape and pose deformation fields to
  * produce a new DomainWithPoseParameters.
  *
  * @see [[DiscreteLowRankGaussianProcess]]
  */
case class ShapeAndPosePGA[DDomain[D] <: DiscreteDomain[D]](
  gp: DiscreteLowRankGaussianProcess[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]],
  logExpMapping: PoseExpLogMappingSingleDomain[DDomain]
)(implicit warperDomainWithPose: DomainWarp[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT],
  warperInnerDomain: DomainWarp[_3D, DDomain],
  rng: scalismo.utils.Random) {

  /**
    * The mean MultiBodyObject
    * @see [[DiscreteLowRankGaussianProcess.mean]]
    */
  val mean = instanceFromDeformationField(gp.instance(DenseVector.zeros[Double](gp.rank)))

  /**
    * returns a DomainWithPoseParameters that corresponds to a combination of the basis functions with the given coefficients coeffs.
    *  @see [[DiscreteLowRankGaussianProcess.instance]]
    */
  def instance(coeffs: DenseVector[Double]): DomainWithPoseParameters[_3D, DDomain] = {
    instanceFromDeformationField(gp.instance(coeffs))
  }
  /**
    * draws a random DomainWithPoseParameters.
    * @see [[DiscreteLowRankGaussianProcess.sample]]
    */
  def sample(): DomainWithPoseParameters[_3D, DDomain] = {
    instanceFromDeformationField(gp.sample())
  }

  def newReference(
    newReference: DomainWithPoseParameters[_3D, DDomain],
    interpolator: FieldInterpolator[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]],
    newLogExpMapping: PoseExpLogMappingSingleDomain[DDomain]
  ): ShapeAndPosePGA[DDomain] = {
    val newGP = gp.interpolate(interpolator).discretize[DomainWithPoseParameters[_3D, DDomain]#DomainT](newReference)
    ShapeAndPosePGA(newGP, newLogExpMapping)
  }


  /**
    *  Returns a marginal ShapeAndPoseGPA over shape features, which a [[PointDistributionModel]]. It models shape deformations only on the given ShapeAndPoseGPA
    *
    */
  def shapePDM: PointDistributionModel[_3D, DDomain] = {
    val interpolatedGP = gp.interpolate(NearestNeighborInterpolator3D())
    val fields = Range(0, interpolatedGP.rank).map(i => {
      val coeffs = DenseVector.zeros[Double](interpolatedGP.rank)
      coeffs(i) = 3.0
      interpolatedGP.instance(coeffs).andThen(_.shapeVec)
    })
    val shapeDeformationGP = DiscreteLowRankGaussianProcess.createUsingPCA[_3D, DDomain, EuclideanVector[_3D]](
      gp.domain.domain,
      fields,
      PivotedCholesky.RelativeTolerance(0)
    )
    PointDistributionModel(shapeDeformationGP)
  }
  /**
    *  Returns a marginal ShapeAndPoseGPA over pose features. It models pose  deformations only on the given ShapeAndPoseGPA
    *
    */
  def PosePGA: ShapeAndPosePGA[DDomain] = {
    val interpolatedGP = gp.interpolate(NearestNeighborInterpolator3D())
    val fields = Range(0, interpolatedGP.rank).map(i => {
      val coeffs = DenseVector.zeros[Double](interpolatedGP.rank)
      coeffs(i) = 3.0
      interpolatedGP.instance(coeffs).andThen(_.copy(shapeVec = EuclideanVector3D.zero))
    })
    val shapeDeformationGP = DiscreteLowRankGaussianProcess
      .createUsingPCA[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
        gp.domain,
        fields,
        PivotedCholesky.RelativeTolerance(0)
      )
    ShapeAndPosePGA(shapeDeformationGP, logExpMapping)
  }
  /**
    * returns a shape that corresponds to a linear combination of the basis functions with the given coefficients.
    */
  private def instanceFromDeformationField(
    shapeAndPoseDF: DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]
  ) = {
    val shapeField = DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
      shapeAndPoseDF.domain,
      shapeAndPoseDF.values.map(_.shapeVec).toIndexedSeq
    )
    val referenceWarped = warperDomainWithPose.transformWithField(gp.domain, shapeField)

    val poseField = DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
      shapeAndPoseDF.domain,
      shapeAndPoseDF.values.map(_.poseVec).toIndexedSeq
    )
    val poseTransforms = logExpMapping.expMappingSingleDomain(poseField)

    warperDomainWithPose.transform(referenceWarped, poseTransforms)
    //referenceWarped

  }
  /**
    * Similar to [[DiscreteLowRankGaussianProcess.posterior(Int, Point[_3D])], sigma2: Double)]],
    * but the training data is defined by specifying the target point instead of the displacement vector
    */
  def posterior(trainingData: IndexedSeq[(PointId, Point[_3D])], sigma2: Double): ShapeAndPosePGA[DDomain] = {

    val trainingDataWithDisplacements = trainingData.map {
      case (id, targetPoint) =>
        (id,
         ShapeAndPoseVector(targetPoint - gp.domain.pointSet.point(id),
                            EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)))
    }

    val posteriorGp = gp.posterior(trainingDataWithDisplacements, sigma2)
    val ExpLog = SinglePoseExpLogMapping(posteriorGp.domain)
    ShapeAndPosePGA(posteriorGp, ExpLog)
  }


  /**
    * transform the ShapeAndPoseGPA using the given rigid transform.
    * The spanned shape and pose space is not affected by this operations.
    */
  def transform(rigidTransform: RigidTransformation[_3D]): ShapeAndPosePGA[DDomain] = {
    val newRef = warperDomainWithPose.transform(gp.domain, rigidTransform)

    val newMean: DenseVector[Double] = {
      val newMeanVecs = (for ((pt, meanAtPoint) <- gp.mean.pointsWithValues) yield {
        Array(rigidTransform(pt + meanAtPoint.shapeVec) - rigidTransform(pt),
          rigidTransform(pt + meanAtPoint.poseVec) - rigidTransform(pt))
      }).flatten
      val data = newMeanVecs.map(_.toArray).flatten.toArray
      DenseVector(data)
    }

    val newBasisMat = DenseMatrix.zeros[Double](gp.basisMatrix.rows, gp.basisMatrix.cols)

    for ((Eigenpair(_, ithKlBasis), i) <- gp.klBasis.zipWithIndex) {
      val newIthBasis = (for ((pt, basisAtPoint) <- ithKlBasis.pointsWithValues) yield {
        Array(rigidTransform(pt + basisAtPoint.shapeVec) - rigidTransform(pt),
          rigidTransform(pt + basisAtPoint.poseVec) - rigidTransform(pt))
      }).flatten
      val data = newIthBasis.map(_.toArray).flatten.toArray
      newBasisMat(::, i) := DenseVector(data)
    }

    val newgp = DiscreteLowRankGaussianProcess[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
      newRef,
      newMean,
      gp.variance,
      newBasisMat
    )
    new ShapeAndPosePGA(newgp, SinglePoseExpLogMapping(newRef))
  }

}

object ShapeAndPosePGA {
  /**
    * Returns a ShapeAndPoseGPA with given a set of items in correspondence after logarithmic computation.
    * All points of the reference domain are considered for computing the PGA
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
    dc: DataCollection[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]
  )(implicit warper: DomainWarp[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT],
    warper2: DomainWarp[_3D, DDomain]): ShapeAndPosePGA[DDomain] = {
    implicit val rng = scalismo.utils.Random(42)
    val singleExpLog = SinglePoseExpLogMapping(dc.reference)

    val gp = DiscreteLowRankGaussianProcess.createUsingPCA(dc)

    ShapeAndPosePGA(gp, singleExpLog)

  }
  /**
    * Creates a new ShapeAndPoseGPA, with its mean and covariance matrix estimated from the given fields.
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
    items: Seq[DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]],
    logExpMapping: PoseExpLogMappingSingleDomain[DDomain]
  )(implicit Warp: DomainWarp[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T],
    warperIner: DomainWarp[_3D, DDomain]): ShapeAndPosePGA[DDomain] = {
    implicit val rng = scalismo.utils.Random(42)

    val dc = new DataCollection(items)

    val gp
      : DiscreteLowRankGaussianProcess[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]] =
      DiscreteLowRankGaussianProcess.createUsingPCA(dc)

    ShapeAndPosePGA(gp, logExpMapping)

  }
  /**
    * creates a ShapeAndPoseGPA from vector/matrix representation of the mean, variance and basis matrix.
    *
    * @see [[DiscreteLowRankGaussianProcess.apply(FiniteDiscreteDomain, DenseVector[Double], DenseVector[Double], DenseMatrix[Double]]
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
    domain: DomainWithPoseParameters[_3D, DDomain],
    meanVector: DenseVector[Double],
    variance: DenseVector[Double],
    basisMatrix: DenseMatrix[Double]
  )(implicit warper: DomainWarp[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT],
    warper2: DomainWarp[_3D, DDomain]): ShapeAndPosePGA[DDomain] = {
    implicit val rng = scalismo.utils.Random(42)

    val gp =
      DiscreteLowRankGaussianProcess[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
        domain,
        meanVector,
        variance,
        basisMatrix
      )
    val expLogMapping = SinglePoseExpLogMapping[DDomain](domain)
    ShapeAndPosePGA(gp, expLogMapping)

  }

}
