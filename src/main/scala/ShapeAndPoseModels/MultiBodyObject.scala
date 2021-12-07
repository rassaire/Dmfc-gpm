package ShapeAndPoseModels

import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common._
import scalismo.geometry._
import scalismo.mesh.{TetrahedralMesh, TriangleMesh}
import scalismo.registration.LandmarkRegistration
import scalismo.transformations._
import scalismo.ui.api.ScalismoUI

/** @param objects: list of discreteDomain
  * @param rotationCenters: list of points
  * @param neutralPoints: list of points
  * Defines a multi body DiscreteDomain with their  associated rotation centers and neutral points.
  * a rotation center is the point that is invariant in the rotation of a DiscreteDomain
  * a neutral point is any point other than the points of the multi body DiscreteDomain and the rotation centers
  */
case class MultiBodyObject[D: NDSpace: UnstructuredPoints.Create, DDomain[D] <: DiscreteDomain[D]](
  val objects: List[DDomain[D]],
  val rotationCenters: List[Point[D]],
  val neutralPoints: List[Point[D]]
)(implicit val warper: DomainWarp[D, DDomain])
    extends DiscreteDomain[D] {

  type DomainT[D] = MultiBodyObject[D, DDomain]

  override def pointSet: PointSet[D] =
    UnstructuredPoints((for (i <- 0 to objects.size - 1) yield {
      objects(i).pointSet.points.toIndexedSeq ++ IndexedSeq(rotationCenters(i)) ++ IndexedSeq(neutralPoints(i))
    }).reduce((acc, s) => acc ++ s))

  /** Rigidly transforms the MultibodyObject and turn the
    * MultibodyObject in new position */
  def transform(transform: List[RigidTransformation[D]]): MultiBodyObject[D, DDomain] = {

    MultiBodyObject[D, DDomain](
      (for (i <- 0 to transform.size - 1) yield {
        warper.transform(objects(i), transform(i))
      }).toList,
      (for (i <- 0 to transform.size - 1) yield {
        transform(i).apply(rotationCenters(i))
      }).toList,
      (for (i <- 0 to transform.size - 1) yield {
        transform(i).apply(neutralPoints(i))
      }).toList
    )
  }

}

object MultiBodyObject {

  class WarperMultiBodyObjects[DDomain[D] <: DiscreteDomain[D]](implicit warper: DomainWarp[_3D, DDomain])
      extends DomainWarp[_3D, MultiBodyObject[_3D, DDomain]#DomainT] {

    /**
      * restricts the DiscreteField over the MultiBodyObject the DiscreteField over a DDomain
      */
    def restrictWarpFieldToSubDomain(
      domain: DDomain[_3D],
      warpField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
    ): DiscreteField[_3D, DDomain, EuclideanVector[_3D]] = {
      val warpFieldInterpolated = warpField.interpolate(NearestNeighborInterpolator())

      val d = DiscreteField(domain, warpFieldInterpolated)
      d
    }

    /**
      * Warp the points of the MultiBodyObject and turn it into the
      * warped DomainWithPoseParameters
      */
    override def transformWithField(
                                     domain: MultiBodyObject[_3D, DDomain],
                                     newWarpField: DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
    ): MultiBodyObject[_3D, DDomain] = {

      val warpField =
        DiscreteField[_3D, MultiBodyObject[_3D, DDomain]#DomainT, EuclideanVector[_3D]](domain, newWarpField.data)
      MultiBodyObject(
        (for (i <- 0 to domain.objects.size - 1) yield {
          val warpFieldOnObject1 = restrictWarpFieldToSubDomain(domain.objects(i), warpField)
          warper.transformWithField(domain.objects(i), warpFieldOnObject1)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          domain.rotationCenters(i)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          domain.neutralPoints(i)
        }).toList
      )
    }
    /** Rigidly transforms the MultiBodyObject and turn the
      * MultiBodyObject in new position */
    override def transform(domain: MultiBodyObject[_3D, DDomain],
                           transformation: Transformation[_3D]): MultiBodyObject[_3D, DDomain] = {

      MultiBodyObject(
        (for (i <- 0 to domain.objects.size - 1) yield {
          warper.transform(domain.objects(i), transformation)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          transformation(domain.rotationCenters(i))
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          transformation(domain.neutralPoints(i))
        }).toList
      )

    }
  }

  implicit val warperMultibodyObjectsTriangleMesh = new WarperMultiBodyObjects[TriangleMesh]()
  implicit val warperMultibodyObjectsTetrahedarlMesh = new WarperMultiBodyObjects[TetrahedralMesh]()

}
