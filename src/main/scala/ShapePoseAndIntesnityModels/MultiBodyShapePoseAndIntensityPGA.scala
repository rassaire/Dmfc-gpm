package ShapePoseAndIntesnityModels

import ShapeAndPoseModels.{MultiBodyObject, MultiBodyShapeAndPosePGA, ShapeAndPoseVector}
import ShapePoseAndIntesnityModels.DomainWithPoseAndIntensity.WarperDomainWithPoseAndIntensity
import ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity.WarperMultiBodyObjectWithIntensity
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.{FieldInterpolator, NearestNeighborInterpolator, NearestNeighborInterpolator3D}
import scalismo.common._
import scalismo.geometry._
import scalismo.numerics.PivotedCholesky
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess.Eigenpair
import scalismo.statisticalmodel.{DiscreteLowRankGaussianProcess, MultivariateNormalDistribution, PointDistributionModel, StatisticalMeshModel}
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.transformations.{RigidTransformation, Transformation}

/**
  * A MultiBodyShapeAndPosePGA models shape and pose variations across given examples of multi body objects as deformation fields using [[DiscreteLowRankGaussianProcess]].
 * the MultiBodyShapePoseAndIntensityPGA while modelling shape deformation fields as [[StatisticalMeshModel]], it uses [[PoseExpLogMapping]] to linearise pose deformation fields.
  * MultiBodyShapePoseAndIntensityPGA warps the MultiBodyObjectWithIntensity with the shape and pose deformation fields to
  * produce a new MultiBodyObjectWithIntensity.
  *
  * @see [[DiscreteLowRankGaussianProcess]]
  */
case class MultiBodyShapePoseAndIntensityPGA[DDomain[D] <: DiscreteDomain[D]](
                                                                      gp: DiscreteLowRankGaussianProcess[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]],
                                                                      logExpMapping: PoseExpLogMapping[DDomain]
)(implicit warperMultiBodyObjectWithIntensity:WarperMultiBodyObjectWithIntensity[DDomain], ultiBodyObjectWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
  warperDomainWithPoseAndIntensity:WarperDomainWithPoseAndIntensity[DDomain], warperInnerDomain: DomainWarp[_3D, DDomain],
  rng: scalismo.utils.Random) {
  /**
    * The mean MultiBodyObjectWithIntensity
    * @see [[DiscreteLowRankGaussianProcess.mean]]
    */
  val mean = instanceFromDeformationField(gp.instance(DenseVector.zeros[Double](gp.rank)))
  /**
    * returns a MultiBodyObjectWithIntensity that corresponds to a combination of the basis functions with the given coefficients coeffs.
    *  @see [[DiscreteLowRankGaussianProcess.instance]]
    */
  def instance(coeffs: DenseVector[Double]): MultiBodyObjectWithIntensity[_3D, DDomain] = {

    instanceFromDeformationField(gp.instance(coeffs))

  }
  /**
    * draws a random MultiBodyObjectWithIntensity.
    * @see [[DiscreteLowRankGaussianProcess.sample]]
    */
  def sample(): MultiBodyObjectWithIntensity[_3D, DDomain] = {
    instanceFromDeformationField(gp.sample())
  }
  /**
    * Warps the reference MultiBodyObjectWithIntensity to a new reference MultiBodyObjectWithIntensity. The space spanned by the model is not affected.
    */
  def newReference(newReference: MultiBodyObjectWithIntensity[_3D, DDomain],
                   interpolator: FieldInterpolator[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]],
                   newLogExpMapping: PoseExpLogMapping[DDomain]): MultiBodyShapePoseAndIntensityPGA[DDomain] = {
    val newGP = gp.interpolate(interpolator).discretize[MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT](newReference)
    MultiBodyShapePoseAndIntensityPGA(newGP, newLogExpMapping)
  }


  def PoseAndPoseModel: MultiBodyShapeAndPosePGA[DDomain] = {
    val interpolatedGP = gp.interpolate(NearestNeighborInterpolator3D())
    val fields = Range(0, interpolatedGP.rank).map(i => {
      val coeffs = DenseVector.zeros[Double](interpolatedGP.rank)
      coeffs(i) = 3.0
      interpolatedGP.instance(coeffs).andThen(g=>ShapeAndPoseVector(g.shapeAndIntensityVec.Vec,g.poseVec))
    })
    val shapeDeformationGP = DiscreteLowRankGaussianProcess
      .createUsingPCA[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
      MultiBodyObject(gp.domain.objects,gp.domain.rotationCenters,gp.domain.neutralPoints),
      fields,
      PivotedCholesky.RelativeTolerance(0)
    )
    val logExp=ShapeAndPoseModels.MultiObjectPoseExpLogMapping(MultiBodyObject(gp.domain.objects,gp.domain.rotationCenters,gp.domain.neutralPoints))
    new MultiBodyShapeAndPosePGA(shapeDeformationGP, logExp)
  }

/**
  *  Returns a marginal MultiBodyShapePoseAndIntensityPGA, which a [[ShapePoseAndIntensityPGA]]. It models deformations only on the given [[DomainWithPoseAndIntensity]]
  *  This method by marginalising over a single body domain return a shape and pose model a single body object.
  *
  */
  def transitionToSingleObject(
    reference: DomainWithPoseAndIntensity[_3D, DDomain],
    logExpMapping: PoseExpLogMappingSingleDomain[DDomain]
  ): ShapePoseAndIntensityPGA[DDomain] = {
    val interpolatedGP = gp.interpolate(NearestNeighborInterpolator3D())

    val newGP = interpolatedGP.discretize[DomainWithPoseAndIntensity[_3D, DDomain]#DomainT](reference)

    ShapePoseAndIntensityPGA(newGP, logExpMapping)
  }

/**
  * returns a shape that corresponds to a linear combination of the basis functions with the given coefficients.
  */
  private def instanceFromDeformationField(
    shapePoseAndIntensityDF: DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]]
  ) = {
    val shapeField = DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]](
      shapePoseAndIntensityDF.domain,
      shapePoseAndIntensityDF.values.map(_.shapeAndIntensityVec).toIndexedSeq
    )
    val referenceWarped = warperMultiBodyObjectWithIntensity.transformWithField(gp.domain, shapeField)

    val poseField = DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
      shapePoseAndIntensityDF.domain,
      shapePoseAndIntensityDF.values.map(_.poseVec).toIndexedSeq
    )
    val poseTransforms = logExpMapping.expMapping(poseField)

    MultiBodyObjectWithIntensity(
      (for (i <- 0 to referenceWarped.objects.size - 1) yield {
        warperInnerDomain.transform(referenceWarped.objects(i), poseTransforms(i))
      }).toList,
      (for (i <- 0 to referenceWarped.objects.size - 1) yield {
       referenceWarped.intensity(i)
      }).toList,
      (for (i <- 0 to referenceWarped.objects.size - 1) yield {
        referenceWarped.rotationCenters(i)
      }).toList,
      (for (i <- 0 to referenceWarped.objects.size - 1) yield {
        referenceWarped.neutralPoints(i)
      }).toList
    )
  }

  /**
    * Similar to [[DiscreteLowRankGaussianProcess.posterior(Int, Point[_3D])], sigma2: Double)]],
    * but the training data is defined by specifying the target point instead of the displacement vector
    */
  def posterior(trainingData: List[IndexedSeq[(PointId, Point[_3D])]],
                sigma2: Double): MultiBodyShapePoseAndIntensityPGA[DDomain] = {

    val trainingDataWithDisplacements = (for (i <- 0 to trainingData.size - 1) yield {
      trainingData(i).map {
        case (id, targetPoint) =>
          (id,
            ShapePoseAndIntensityVector[_3D](PointWithIntensityVectorVector(targetPoint - gp.domain.pointSet.point(id),Double.NaN),
                              EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)))
      }
    }).reduce((acc, s) => acc ++ s)
    val posteriorGp = gp.posterior(trainingDataWithDisplacements, sigma2)
    val ExpLog = MultiObjectPosewithIntensityExpLogMapping(posteriorGp.domain)
    MultiBodyShapePoseAndIntensityPGA(posteriorGp, ExpLog)
  }
  /**
    * Similar to [[DiscreteLowRankGaussianProcess.posterior(Int, Point[_3D], Double)]]],
    * but the training data is defined by specifying the target point instead of the displacement vector
    */
  def posterior(
    trainingData: List[IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)]]
  ): MultiBodyShapePoseAndIntensityPGA[DDomain] = {
    val trainingDataWithDisplacements = (for (i <- 0 to trainingData.size - 1) yield {
      trainingData(i).map {
        case (id, targetPoint, cov) =>
          (id,
            ShapePoseAndIntensityVector[_3D](PointWithIntensityVectorVector(targetPoint - gp.domain.pointSet.point(id),Double.NaN),
              EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)),
           cov)
      }
    }).reduce((acc, s) => acc ++ s)

    val posteriorGp = gp.posterior(trainingDataWithDisplacements)
    MultiBodyShapePoseAndIntensityPGA(posteriorGp, logExpMapping)
  }

  /**
    * transform the MultiBodyShapePoseAndIntensityPGA using the given rigid transform.
    * The spanned shape and pose space is not affected by this operations.
    */
  def transform(rigidTransform: RigidTransformation[_3D]): MultiBodyShapePoseAndIntensityPGA[DDomain] = {
    val newRef = warperMultiBodyObjectWithIntensity.transform(gp.domain, rigidTransform)

    val newMean: DenseVector[Double] = {
      val newMeanVecs = (for ((pt, meanAtPoint) <- gp.mean.pointsWithValues) yield {
        Array(rigidTransform(pt + meanAtPoint.shapeAndIntensityVec.Vec) - rigidTransform(pt),
              rigidTransform(pt + meanAtPoint.poseVec) - rigidTransform(pt))
      }).flatten
      val data = newMeanVecs.map(_.toArray).flatten.toArray
      DenseVector(data)
    }

    val newBasisMat = DenseMatrix.zeros[Double](gp.basisMatrix.rows, gp.basisMatrix.cols)

    for ((Eigenpair(_, ithKlBasis), i) <- gp.klBasis.zipWithIndex) {
      val newIthBasis = (for ((pt, basisAtPoint) <- ithKlBasis.pointsWithValues) yield {
        Array(rigidTransform(pt + basisAtPoint.shapeAndIntensityVec.Vec) - rigidTransform(pt),
              rigidTransform(pt + basisAtPoint.poseVec) - rigidTransform(pt))
      }).flatten
      val data = newIthBasis.map(_.toArray).flatten.toArray
      newBasisMat(::, i) := DenseVector(data)
    }

    val newgp = DiscreteLowRankGaussianProcess[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]](
      newRef,
      newMean,
      gp.variance,
      newBasisMat
    )
    new MultiBodyShapePoseAndIntensityPGA(newgp, MultiObjectPosewithIntensityExpLogMapping(newRef))
  }

}

object MultiBodyShapePoseAndIntensityPGA {
  /**
    * Returns a MultiBodyShapePoseAndIntensityPGA with given a set of items in correspondence after logarithmic computation.
    * All points of the reference domain are considered for computing the PGA
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
    dc: DataCollection[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]]
  )(implicit warperMultiBodyObjectWithIntensity: WarperMultiBodyObjectWithIntensity[DDomain], multiBodyObjectWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    warperDomainWithPoseAndIntensity:WarperDomainWithPoseAndIntensity[DDomain], domainWarper: DomainWarp[_3D, DDomain],
    rng: scalismo.utils.Random): MultiBodyShapePoseAndIntensityPGA[DDomain] = {

    val singleExpLog = MultiObjectPosewithIntensityExpLogMapping(dc.reference)

    val gp: DiscreteLowRankGaussianProcess[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]] =
      DiscreteLowRankGaussianProcess.createUsingPCA(dc)

    MultiBodyShapePoseAndIntensityPGA(gp, singleExpLog)

  }


  /**
    * creates a MultiBodyShapePoseAndIntensityPGA from vector/matrix representation of the mean, variance and basis matrix.
    *
    * @see [[DiscreteLowRankGaussianProcess.apply(FiniteDiscreteDomain, DenseVector[Double], DenseVector[Double], DenseMatrix[Double]]
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
                                              domain: MultiBodyObjectWithIntensity[_3D, DDomain],
                                              meanVector: DenseVector[Double],
                                              variance: DenseVector[Double],
                                              basisMatrix: DenseMatrix[Double]
  )(implicit warperMultiBodyObjectWithIntensity: WarperMultiBodyObjectWithIntensity[DDomain], multiBodyObjectWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    warperDomainWithPoseAndIntensit:WarperDomainWithPoseAndIntensity[DDomain], domainWarper: DomainWarp[_3D, DDomain]): MultiBodyShapePoseAndIntensityPGA[DDomain] = {
    implicit val rng = scalismo.utils.Random(42)
    val gp = DiscreteLowRankGaussianProcess[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]](
      domain,
      meanVector,
      variance,
      basisMatrix
    )
    val expLogMapping = MultiObjectPosewithIntensityExpLogMapping[DDomain](domain)
    new MultiBodyShapePoseAndIntensityPGA[DDomain](gp, expLogMapping)
  }


  /**
    * Creates a new  MultiBodyShapePoseAndIntensityPGA, with its mean and covariance matrix estimated from the given fields.
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
                                              items: Seq[DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]]],
                                              logExpMapping: PoseExpLogMapping[DDomain]
  )(implicit warperMultiBodyObjectWithIntensity: WarperMultiBodyObjectWithIntensity[DDomain], multiBodyObjectWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    warperDomainWithPoseAndIntensity:WarperDomainWithPoseAndIntensity[DDomain], domainWarper: DomainWarp[_3D, DDomain],
    rng: scalismo.utils.Random): MultiBodyShapePoseAndIntensityPGA[DDomain] = {

    val dc = new DataCollection(items)
    val gp = DiscreteLowRankGaussianProcess.createUsingPCA(dc)
    MultiBodyShapePoseAndIntensityPGA(gp, logExpMapping)

  }
}
