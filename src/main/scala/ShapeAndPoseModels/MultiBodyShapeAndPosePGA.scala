package ShapeAndPoseModels

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.{FieldInterpolator, NearestNeighborInterpolator, NearestNeighborInterpolator3D}
import scalismo.common._
import scalismo.geometry._
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess.Eigenpair
import scalismo.statisticalmodel.{
  DiscreteLowRankGaussianProcess,
  MultivariateNormalDistribution,
  PointDistributionModel,
  StatisticalMeshModel
}
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.transformations.{RigidTransformation, Transformation}

/**
  * A MultiBodyShapeAndPosePGA models shape and pose variations across given examples of multi body objects as deformation fields using [[DiscreteLowRankGaussianProcess]].
 * the MultiBodyShapeAndPosePGA while modelling shape deformation fields as [[StatisticalMeshModel]], it uses [[PoseExpLogMapping]] to linearise pose deformation fields.
  * MultiBodyShapeAndPosePGA warps the MultiBodyObject with the shape and pose deformation fields to
  * produce a new MultiBodyObject.
  *
  * @see [[DiscreteLowRankGaussianProcess]]
  */
case class MultiBodyShapeAndPosePGA[DDomain[D] <: DiscreteDomain[D]](
                                                                      gp: DiscreteLowRankGaussianProcess[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]],
                                                                      logExpMapping: PoseExpLogMapping[DDomain]
)(implicit warperMBObject: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
  warperInnerDomain: DomainWarp[_3D, DDomain],
  rng: scalismo.utils.Random) {
  /**
    * The mean MultiBodyObject
    * @see [[DiscreteLowRankGaussianProcess.mean]]
    */
  val mean = instanceFromDeformationField(gp.instance(DenseVector.zeros[Double](gp.rank)))
  /**
    * returns a MultiBodyObject that corresponds to a combination of the basis functions with the given coefficients coeffs.
    *  @see [[DiscreteLowRankGaussianProcess.instance]]
    */
  def instance(coeffs: DenseVector[Double]): MultiBodyObject[_3D, DDomain] = {

    instanceFromDeformationField(gp.instance(coeffs))

  }
  /**
    * draws a random MultiBodyObject.
    * @see [[DiscreteLowRankGaussianProcess.sample]]
    */
  def sample(): MultiBodyObject[_3D, DDomain] = {
    instanceFromDeformationField(gp.sample())
  }
  /**
    * Warps the reference MultiBodyObject to a new reference MultiBodyObject. The space spanned by the model is not affected.
    */
  def newReference(newReference: MultiBodyObject[_3D, DDomain],
                   interpolator: FieldInterpolator[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]],
                   newLogExpMapping: PoseExpLogMapping[DDomain]): MultiBodyShapeAndPosePGA[DDomain] = {
    val newGP = gp.interpolate(interpolator).discretize[MultiBodyObject[_3D, DDomain]#DomainT](newReference)
    MultiBodyShapeAndPosePGA(newGP, newLogExpMapping)
  }

/**
  *  Returns a marginal MultiBodyShapeAndPosePGA, which a [[ShapeAndPosePGA]]. It models deformations only on the given [[DomainWithPoseParameters]]
  *  This method by marginalising over a single body domain return a shape and pose model a single body object.
  *
  */
  def transitionToSingleObject(
    reference: DomainWithPoseParameters[_3D, DDomain],
    logExpMapping: PoseExpLogMappingSingleDomain[DDomain]
  )(implicit warper: DomainWarp[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT]): ShapeAndPosePGA[DDomain] = {
    val interpolatedGP = gp.interpolate(NearestNeighborInterpolator3D())

    val newGP = interpolatedGP.discretize[DomainWithPoseParameters[_3D, DDomain]#DomainT](reference)

    ShapeAndPosePGA(newGP, logExpMapping)
  }

/**
  * returns a shape that corresponds to a linear combination of the basis functions with the given coefficients.
  */
  private def instanceFromDeformationField(
    shapeAndPoseDF: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]
  ) = {
    val shapeField = DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
      shapeAndPoseDF.domain,
      shapeAndPoseDF.values.map(_.shapeVec).toIndexedSeq
    )
    val referenceWarped = warperMBObject.transformWithField(gp.domain, shapeField)

    val poseField = DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
      shapeAndPoseDF.domain,
      shapeAndPoseDF.values.map(_.poseVec).toIndexedSeq
    )
    val poseTransforms = logExpMapping.expMapping(poseField)

    MultiBodyObject(
      (for (i <- 0 to referenceWarped.objects.size - 1) yield {
        warperInnerDomain.transform(referenceWarped.objects(i), poseTransforms(i))
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
                sigma2: Double): MultiBodyShapeAndPosePGA[DDomain] = {

    val trainingDataWithDisplacements = (for (i <- 0 to trainingData.size - 1) yield {
      trainingData(i).map {
        case (id, targetPoint) =>
          (id,
           ShapeAndPoseVector(targetPoint - gp.domain.pointSet.point(id),
                              EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)))
      }
    }).reduce((acc, s) => acc ++ s)
    val posteriorGp = gp.posterior(trainingDataWithDisplacements, sigma2)
    val ExpLog = MultiObjectPoseExpLogMapping(posteriorGp.domain)
    MultiBodyShapeAndPosePGA(posteriorGp, ExpLog)
  }
  /**
    * Similar to [[DiscreteLowRankGaussianProcess.posterior(Int, Point[_3D], Double)]]],
    * but the training data is defined by specifying the target point instead of the displacement vector
    */
  def posterior(
    trainingData: List[IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)]]
  ): MultiBodyShapeAndPosePGA[DDomain] = {
    val trainingDataWithDisplacements = (for (i <- 0 to trainingData.size - 1) yield {
      trainingData(i).map {
        case (id, targetPoint, cov) =>
          (id,
           ShapeAndPoseVector(targetPoint - gp.domain.objects(i).pointSet.point(id),
                              EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)),
           cov)
      }
    }).reduce((acc, s) => acc ++ s)

    val posteriorGp = gp.posterior(trainingDataWithDisplacements)
    MultiBodyShapeAndPosePGA(posteriorGp, logExpMapping)
  }

  /**
    * transform the MultiBodyShapeAndPosePGA using the given rigid transform.
    * The spanned shape and pose space is not affected by this operations.
    */
  def transform(rigidTransform: RigidTransformation[_3D]): MultiBodyShapeAndPosePGA[DDomain] = {
    val newRef = warperMBObject.transform(gp.domain, rigidTransform)

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

    val newgp = DiscreteLowRankGaussianProcess[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
      newRef,
      newMean,
      gp.variance,
      newBasisMat
    )
    new MultiBodyShapeAndPosePGA(newgp, MultiObjectPoseExpLogMapping(newRef))
  }

}

object MultiBodyShapeAndPosePGA {
  /**
    * Returns a MultiBodyShapeAndPosePGA with given a set of items in correspondence after logarithmic computation.
    * All points of the reference domain are considered for computing the PGA
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
    dc: DataCollection[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]
  )(implicit multiiBodyWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    innerWarper: DomainWarp[_3D, DDomain],
    rng: scalismo.utils.Random): MultiBodyShapeAndPosePGA[DDomain] = {

    val singleExpLog = MultiObjectPoseExpLogMapping(dc.reference)

    val gp: DiscreteLowRankGaussianProcess[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]] =
      DiscreteLowRankGaussianProcess.createUsingPCA(dc)

    MultiBodyShapeAndPosePGA(gp, singleExpLog)

  }
  /**
    * creates a MultiBodyShapeAndPosePGA from vector/matrix representation of the mean, variance and basis matrix.
    *
    * @see [[DiscreteLowRankGaussianProcess.apply(FiniteDiscreteDomain, DenseVector[Double], DenseVector[Double], DenseMatrix[Double]]
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
                                              domain: MultiBodyObject[_3D, DDomain],
                                              meanVector: DenseVector[Double],
                                              variance: DenseVector[Double],
                                              basisMatrix: DenseMatrix[Double]
  )(implicit warper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    warper2: DomainWarp[_3D, DDomain]): MultiBodyShapeAndPosePGA[DDomain] = {
    implicit val rng = scalismo.utils.Random(42)
    val gp = DiscreteLowRankGaussianProcess[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](
      domain,
      meanVector,
      variance,
      basisMatrix
    )
    val expLogMapping = MultiObjectPoseExpLogMapping[DDomain](domain)
    new MultiBodyShapeAndPosePGA[DDomain](gp, expLogMapping)
  }


  /**
    * Creates a new  MultiBodyShapeAndPosePGA, with its mean and covariance matrix estimated from the given fields.
    *
    * Per default, the resulting multi body model will have rank (i.e. number of principal geodesic) corresponding to
    * the number of linear independent fields at the tangent space.
    */
  def apply[DDomain[D] <: DiscreteDomain[D]](
                                              items: Seq[DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]],
                                              logExpMapping: PoseExpLogMapping[DDomain]
  )(implicit multibodyObjectWarper: DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT],
    domainWarper: DomainWarp[_3D, DDomain],
    rng: scalismo.utils.Random): MultiBodyShapeAndPosePGA[DDomain] = {

    val dc = new DataCollection(items)
    val gp = DiscreteLowRankGaussianProcess.createUsingPCA(dc)
    MultiBodyShapeAndPosePGA(gp, logExpMapping)

  }
}
