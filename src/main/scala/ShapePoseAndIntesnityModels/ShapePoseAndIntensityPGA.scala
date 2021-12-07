package ShapePoseAndIntesnityModels

import ShapePoseAndIntesnityModels.DomainWithPoseAndIntensity.WarperDomainWithPoseAndIntensity
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.{FieldInterpolator, NearestNeighborInterpolator3D}
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, PointId}
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Point, _3D}
import scalismo.numerics.PivotedCholesky
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess.Eigenpair
import scalismo.statisticalmodel.{DiscreteLowRankGaussianProcess, PointDistributionModel}
import scalismo.transformations.RigidTransformation

/**
  * A ShapeAndPoseGPA models shape and pose variations across given examples of mesh with Pose Parameters as deformation fields using [[DiscreteLowRankGaussianProcess]].
  * the ShapeAndPoseGPA while modelling shape deformation fields as [[scalismo.statisticalmodel.StatisticalMeshModel]], it uses [[PoseExpLogMapping]] to linearise pose deformation fields.
  * ShapeAndPoseGPA warps the single body object with the shape and pose deformation fields to
  * produce a new DomainWithPoseAndIntensity.
  *
  * @see [[DiscreteLowRankGaussianProcess]]
  */
case class ShapePoseAndIntensityPGA[DDomain[D] <: DiscreteDomain[D]](
  gp: DiscreteLowRankGaussianProcess[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]],
  logExpMapping: PoseExpLogMappingSingleDomain[DDomain]
)(implicit warperDomainWithPose:WarperDomainWithPoseAndIntensity[DDomain],
  warperInnerDomain: DomainWarp[_3D, DDomain],
  rng: scalismo.utils.Random) {

  /**
    * The mean MultiBodyObject
    * @see [[DiscreteLowRankGaussianProcess.mean]]
    */
  val mean = instanceFromDeformationField(gp.instance(DenseVector.zeros[Double](gp.rank)))

  /**
    * returns a DomainWithPoseAndIntensity that corresponds to a combination of the basis functions with the given coefficients coeffs.
    *  @see [[DiscreteLowRankGaussianProcess.instance]]
    */
  def instance(coeffs: DenseVector[Double]): DomainWithPoseAndIntensity[_3D, DDomain] = {
    instanceFromDeformationField(gp.instance(coeffs))
  }
  /**
    * draws a random DomainWithPoseAndIntensity.
    * @see [[DiscreteLowRankGaussianProcess.sample]]
    */
  def sample(): DomainWithPoseAndIntensity[_3D, DDomain] = {
    instanceFromDeformationField(gp.sample())
  }

  def newReference(
    newReference: DomainWithPoseAndIntensity[_3D, DDomain],
    interpolator: FieldInterpolator[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]],
    newLogExpMapping: PoseExpLogMappingSingleDomain[DDomain]
  ): ShapePoseAndIntensityPGA[DDomain] = {
    val newGP = gp.interpolate(interpolator).discretize[DomainWithPoseAndIntensity[_3D, DDomain]#DomainT](newReference)
    ShapePoseAndIntensityPGA(newGP, newLogExpMapping)
  }

  /**
    * returns a shape that corresponds to a linear combination of the basis functions with the given coefficients.
    */
  private def instanceFromDeformationField(
    shapeAndPoseDF: DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]]
  ) = {
    val shapeField = DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]](
      shapeAndPoseDF.domain,
      shapeAndPoseDF.values.map(_.shapeAndIntensityVec).toIndexedSeq
    )
    val referenceWarped = warperDomainWithPose.transformWithField(gp.domain, shapeField)

    val poseField = DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
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
  def posterior(trainingData: IndexedSeq[(PointId, Point[_3D])], sigma2: Double): ShapePoseAndIntensityPGA[DDomain] = {

    val trainingDataWithDisplacements = trainingData.map {
      case (id, targetPoint) =>
        (id,
         ShapePoseAndIntesnityModels.ShapePoseAndIntensityVector(PointWithIntensityVectorVector(targetPoint - gp.domain.pointSet.point(id),Double.NaN),
                            EuclideanVector3D(Double.NaN, Double.NaN, Double.NaN)))
    }

    val posteriorGp = gp.posterior(trainingDataWithDisplacements, sigma2)
    val ExpLog = SinglePoseExpLogMapping(posteriorGp.domain)
    ShapePoseAndIntensityPGA(posteriorGp, ExpLog)
  }


  /**
    * transform the ShapeAndPoseGPA using the given rigid transform.
    * The spanned shape and pose space is not affected by this operations.
    */
  def transform(rigidTransform: RigidTransformation[_3D]): ShapePoseAndIntensityPGA[DDomain] = {
    val newRef = warperDomainWithPose.transform(gp.domain, rigidTransform)

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

    val newgp = DiscreteLowRankGaussianProcess[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, ShapePoseAndIntensityVector[_3D]](
      newRef,
      newMean,
      gp.variance,
      newBasisMat
    )
    new ShapePoseAndIntensityPGA(newgp, SinglePoseExpLogMapping(newRef))
  }

}


