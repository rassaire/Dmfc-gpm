package ShapeAndPoseModels

import breeze.linalg.DenseVector
import breeze.stats.distributions.Uniform
import scalismo.common._
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{_3D, _}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess
import scalismo.transformations._
import scalismo.ui.api.ScalismoUI

import scala.util.{Failure, Success}

/**
  *
  * defines the logarithmic  and exponential mappings for multi body objects
  */
trait PoseExpLogMapping[DDomain[D] <: DiscreteDomain[D]] {

  def expMapping(
    deformationField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
  ): List[Transformation[_3D]]

  def logMapping(
    deformationField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
  ): DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]]

}
/** @param domain: a [[MultiBodyObject]]
  * Defines the logarithmic and the exponential mapping for a given reference MultiBodyObject.
  * The logarithmic mapping projects pose variations from the manifold to the tangent space at reference
  * the exponential mapping is the inverse of the logarithmic
  */
case class MultiObjectPoseExpLogMapping[DDomain[D] <: DiscreteDomain[D]](domain: MultiBodyObject[_3D, DDomain])(
  implicit warper: DomainWarp[_3D, DDomain]
) extends PoseExpLogMapping[DDomain] {
  /**
    * restricts the DiscreteField over the MultiBodyObject the DiscreteField over a DDomain
    */
  private def restrictWarpFieldToSubdomain(
    domain: DDomain[_3D],
    warpField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
  ): DiscreteField[_3D, DDomain, EuclideanVector[_3D]] = {

    val field = warpField.interpolate(NearestNeighborInterpolator())
    field.discretize(domain, EuclideanVector.zeros[_3D]) // outside value is not accessed. If it is, it should be an error
  }

  /**
   *
   * @param deformationField: a DiscreteField
   * @return a list  rigid transforms
   */
  override def expMapping(
    deformationField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
  ): List[Transformation[_3D]] = {

    implicit val rand = scalismo.utils.Random(42)

    val l = (for (i <- 0 to deformationField.domain.objects.size - 1) yield {
      val reference = deformationField.domain.objects(i)
      val restrictedDF = restrictWarpFieldToSubdomain(reference, deformationField)

      val points = reference.pointSet.points.toIndexedSeq
      val distrDim1 = Uniform(0, reference.pointSet.numberOfPoints)(rand.breezeRandBasis)
      val referencePoints = (0 until 50).map(i => points(distrDim1.draw().toInt))
      val referenceLandmarks = referencePoints.map { p =>
        val pi = reference.pointSet.findClosestPoint(p)
        (Landmark[_3D](pi.id.toString, pi.point), pi.id)
      }

      val warpedLandmark = referenceLandmarks
        .map(pi => Landmark[_3D](pi._1.id, pi._1.point + restrictedDF(pi._2)))
        .toSeq
      val warpedRotationCenter =
        deformationField.interpolate(NearestNeighborInterpolator())(deformationField.domain.rotationCenters(i)).toPoint
      val warpedNeutralPoint =
        deformationField.interpolate(NearestNeighborInterpolator())(deformationField.domain.neutralPoints(i)).toPoint

      val bestTransform = LandmarkRegistration.rigid3DLandmarkRegistration(referenceLandmarks.map(p => p._1),
                                                                           warpedLandmark,
                                                                           deformationField.domain.rotationCenters(i))

      //compute the pose transformation of the new i^th shape, which obtained from the rigid transform
      val newTranslation = Translation3D(warpedRotationCenter - warpedNeutralPoint)
      RotationAfterTranslation3D(newTranslation, bestTransform.rotation)

    }).toList

    l

  }

  /**
   *
   * @param df: a DiscreteField  (ths discreteField maps the domain onto  the target multiBodyObjects domain: here shape and pose features are still combined)
   * @return a discreteFiled, where shape and pose feature are factorized
   *         The returned  pose here is in the "biomechanical" sens, that is, the translation is related to rotation center or
   *         specific landmark provided(neutralPoseRefPointsIds), and not to center of mass.
   */
  override def logMapping(
    df: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
  ): DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]] = {

    //computation of the list of pose parameters needed for log deformation field of the MultiBodyObject
    val transformations = for (i <- 0 to df.domain.objects.size - 1) yield {
      val reference = df.domain.objects(i)
      val restrictedDF = restrictWarpFieldToSubdomain(reference, df)
      val targMesh = warper.transformWithField(domain.objects(i), restrictedDF)
      //compute landmarks and rotation center for the rigid transformation computation
      val referenceLandmark =
        reference.pointSet.pointIds.map(pi => Landmark[_3D](pi.id.toString, reference.pointSet.point(pi))).toSeq
      val targetLandmark =
        reference.pointSet.pointsWithId.map(pi => Landmark[_3D](pi._2.id.toString, pi._1 + restrictedDF(pi._2))).toSeq
      val targetRotationCenter = df.domain.rotationCenters(i) + df.interpolate(NearestNeighborInterpolator())(
        df.domain.rotationCenters(i)
      )
      val bestTransform =
        LandmarkRegistration.rigid3DLandmarkRegistration(targetLandmark, referenceLandmark, targetRotationCenter)
      //compute the pose transformation (then pose deformation field), which obtained from the rigid transform
      val translation = Translation3D(df.domain.rotationCenters(i) - targetRotationCenter)
      val poseTransform = TranslationAfterRotation3D(translation, bestTransform.rotation)

      val poseDF = DiscreteField(
        reference,
        reference.pointSet.points.map(pt => poseTransform.inverse.apply(pt) - pt).toIndexedSeq
      )
      //compute shape deformation field
      val shapeDF = DiscreteField(reference,
                                  reference.pointSet.pointsWithId
                                    .map(pt => bestTransform.apply(targMesh.pointSet.point(pt._2)) - pt._1)
                                    .toIndexedSeq)
      //compute the neutral point required for the retrieval of the neutral position of the shape
      val targetNeutralPoint = bestTransform(
        df.domain.neutralPoints(i) + df.interpolate(NearestNeighborInterpolator())(df.domain.neutralPoints(i))
      )
      (shapeDF, poseDF, targetRotationCenter, targetNeutralPoint)
    }
    //computation of the log vector fields, with associated vector having shape and pose components
    val data = (for (i <- 0 to transformations.size - 1) yield {
      transformations(i)._1.pointsWithIds
        .map(pi => ShapeAndPoseVector(transformations(i)._1(PointId(pi._2)), transformations(i)._2(PointId(pi._2))))
        .toIndexedSeq ++
        IndexedSeq(
          ShapeAndPoseVector((transformations(i)._3 - df.domain.rotationCenters(i)),
                             (transformations(i)._3 - df.domain.rotationCenters(i)))
        ) ++
        IndexedSeq(
          ShapeAndPoseVector((transformations(i)._4 - df.domain.neutralPoints(i)),
                             (transformations(i)._4 - df.domain.neutralPoints(i)))
        )
    }).reduce((acc, s) => acc ++ s)
    DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, ShapeAndPoseVector[_3D]](df.domain, data)

  }

}



/**
  *
  * defines the logarithmic  and exponential mappings for DomainWithPoseParameters
  */
trait PoseExpLogMappingSingleDomain[DDomain[D] <: DiscreteDomain[D]] {

  def expMappingSingleDomain(
    deformationField: DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, EuclideanVector[_3D]]
  ): Transformation[_3D]
  def logMappingSingleDomain(
    deformationField: DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, EuclideanVector[_3D]]
  ): DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, ShapeAndPoseVector[_3D]]

}
/**
  *
  * @param deformationField: a DiscreteField
  * @return a rigid transforms
  */
case class SinglePoseExpLogMapping[DDomain[D] <: DiscreteDomain[D]](reference: DomainWithPoseParameters[_3D, DDomain])(
  implicit warper: DomainWarp[_3D, DDomain]
) extends PoseExpLogMappingSingleDomain[DDomain] {

  implicit val rand = scalismo.utils.Random(42)

  /**
    * restricts the DiscreteField over the DomainWithPoseParameters to the DiscreteField over the  DDomain
    */
  override def expMappingSingleDomain(
    deformationField: DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, EuclideanVector[_3D]]
  ): Transformation[_3D] = {

    val warpFieldInterpolated = deformationField.interpolate(NearestNeighborInterpolator())
    val restrictedDF = DiscreteField(reference.domain, warpFieldInterpolated)

    //compute landmarks and rotation center for the rigid transformation computation
    val points = reference.pointSet.points.toIndexedSeq
    val distrDim1 = Uniform(0, reference.pointSet.numberOfPoints)(rand.breezeRandBasis)
    val referencePoints = (0 until 50).map(i => points(distrDim1.draw().toInt))
    val referenceLandmarks = referencePoints.map { p =>
      val pi = reference.pointSet.findClosestPoint(p)
      (Landmark[_3D](pi.id.toString, pi.point), pi.id)
    }

    val warpedLandmark = referenceLandmarks
      .map(pi => Landmark[_3D](pi._1.id, pi._1.point + restrictedDF(pi._2)))
      .toSeq
    val warpedRotationCenter: Point[_3D] = warpFieldInterpolated(reference.rotCenter).toPoint
    val warpedNeutralPoint: Point[_3D] = warpFieldInterpolated(reference.neutralPoint).toPoint

    val bestTransform: TranslationAfterRotation[_3D] =
      LandmarkRegistration.rigid3DLandmarkRegistration(referenceLandmarks.map(l => l._1),
                                                       warpedLandmark,
                                                       reference.rotCenter)

    //compute the pose transformation of the new shape, which is obtained from the rigid transform
    val newTranslation = Translation3D(warpedRotationCenter - warpedNeutralPoint)
    RotationAfterTranslation3D(newTranslation, bestTransform.rotation)
  }
  /**
    *
    * @param df: a DiscreteField  (ths discreteField maps the domain onto  the target DomainWithPoseParameters: here shape and pose features are still combined)
    * @return a discreteFiled, where shape and pose feature are factorized
    *         The returned  pose here is in the "biomechanical" sens, that is, the translation is related to rotation center or
    *         specific landmark provided(neutralPoseRefPointsIds), and not to center of mass.
    */
  override def logMappingSingleDomain(
    deformationField: DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, EuclideanVector[_3D]]
  ): DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, ShapeAndPoseVector[_3D]] = {

    val warpFieldInterpolated = deformationField.interpolate(NearestNeighborInterpolator())

    val restrictedDF = DiscreteField(reference.domain, warpFieldInterpolated)

    val targMesh = warper.transformWithField(reference.domain, restrictedDF)

    //compute landmarks and rotation center for the rigid transformation computation
    val referenceLandmark = reference.domain.pointSet.pointIds
      .map(pi => Landmark[_3D](pi.id.toString, reference.domain.pointSet.point(pi)))
      .toSeq
    val targetLandmark = reference.domain.pointSet.pointsWithId
      .map(pi => Landmark[_3D](pi._2.id.toString, pi._1 + restrictedDF(pi._2)))
      .toSeq
    val targetRotationCenter = reference.rotCenter + warpFieldInterpolated(reference.rotCenter)
    val bestTransform =
      LandmarkRegistration.rigid3DLandmarkRegistration(targetLandmark, referenceLandmark, targetRotationCenter)

    //compute the pose transformation (then pose deformation field), which obtained from the rigid transform
    val translation = Translation3D(reference.rotCenter - targetRotationCenter)
    val poseTransform = TranslationAfterRotation3D(translation, bestTransform.rotation)

    val poseDF = DiscreteField(
      reference.domain,
      reference.domain.pointSet.points.map(pt => poseTransform.inverse.apply(pt) - pt).toIndexedSeq
    )
    //compute shape deformation field
    val shapeDF = DiscreteField(reference.domain,
                                reference.domain.pointSet.pointsWithId
                                  .map(pt => bestTransform.apply(targMesh.pointSet.point(pt._2)) - pt._1)
                                  .toIndexedSeq)
    //compute the neutral point required for the retrieval of the neutral position of the shape
    val targetNeutralPoint = bestTransform(reference.rotCenter + warpFieldInterpolated(reference.rotCenter))

    //computation of the log vector fields, with associated vector having shape and pose components
    val data = shapeDF.pointsWithIds
      .map(pi => ShapeAndPoseVector(shapeDF(PointId(pi._2)), poseDF(PointId(pi._2))))
      .toIndexedSeq ++
      IndexedSeq(
        ShapeAndPoseVector(targetRotationCenter - reference.rotCenter, targetRotationCenter - reference.rotCenter)
      ) ++
      IndexedSeq(
        ShapeAndPoseVector(targetNeutralPoint - reference.neutralPoint, targetNeutralPoint - reference.neutralPoint)
      )

    DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, DDomain] })#T, ShapeAndPoseVector[_3D]](
      deformationField.domain,
      data
    )
  }
}