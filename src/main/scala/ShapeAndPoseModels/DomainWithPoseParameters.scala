package ShapeAndPoseModels

import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, PointSet, UnstructuredPoints}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{_3D, EuclideanVector, NDSpace, Point}
import scalismo.mesh.{TetrahedralMesh, TriangleMesh}
import scalismo.transformations.{RigidTransformation, Transformation}
/** @param domain: a discreteDomain
  * @param rotCenter: a point
  * @param neutralPoint: a point
  * Defines DiscreteDomain with its associated rotation center  and neutral points.
  * the rotation center is the point that is invariant in the rotation of the DiscreteDomain
  * the neutral point is any point other than the points of the DiscreteDomain and the rotation center
  */
case class DomainWithPoseParameters[D: NDSpace: UnstructuredPoints.Create, DDomain[D] <: DiscreteDomain[D]](
  domain: DDomain[D],
  rotCenter: Point[D],
  neutralPoint: Point[D]
)(implicit val warper: DomainWarp[D, DDomain])
    extends DiscreteDomain[D] {

  type DomainT[D] = DomainWithPoseParameters[D, DDomain]

  override def pointSet: PointSet[D] = UnstructuredPoints(
    domain.pointSet.points.toIndexedSeq ++ IndexedSeq(rotCenter) ++ IndexedSeq(neutralPoint)
  )

  /** Rigidly transforms the DomainWithPoseParameters and turn the
    * DomainWithPoseParameters in new position */
  def transform(transform: RigidTransformation[D]): DomainWithPoseParameters[D, DDomain] = {
    DomainWithPoseParameters[D, DDomain](
      warper.transform(domain, transform),
      transform.apply(rotCenter),
      transform.apply(neutralPoint)
    )
  }

}

object DomainWithPoseParameters {

  class WarperDomainWithPoseParameters[DDomain[D] <: DiscreteDomain[D]](implicit warper: DomainWarp[_3D, DDomain])
      extends DomainWarp[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT] {
    /**
      * restricts the DiscreteField over the DomainWithPoseParameters the DiscreteField over the DDomain
      */
    def restrictWarpFieldTodomain(
      domain: DomainWithPoseParameters[_3D, DDomain],
      warpField: DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
    ): (DiscreteField[_3D, DDomain, EuclideanVector[_3D]], Point[_3D], Point[_3D]) = {
      val warpFieldInterpolated = warpField.interpolate(NearestNeighborInterpolator())

      val d: DiscreteField[_3D, DDomain, EuclideanVector[_3D]] = DiscreteField(domain.domain, warpFieldInterpolated)
      val rotCenter = warpField.domain.rotCenter + warpFieldInterpolated(warpField.domain.rotCenter)
      val neutralPoint = warpField.domain.neutralPoint + warpFieldInterpolated(warpField.domain.neutralPoint)
      (d, rotCenter, neutralPoint)
    }
    /**
      * Warp the points of the DomainWithPoseParameters and turn it into the
      * warped DomainWithPoseParameters
      */
    override def transformWithField(
      domain: DomainWithPoseParameters[_3D, DDomain],
      newWarpField: DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, EuclideanVector[_3D]]
    ): DomainWithPoseParameters[_3D, DDomain] = {

      val df = DiscreteField[_3D, DomainWithPoseParameters[_3D, DDomain]#DomainT, EuclideanVector[_3D]](
        domain,
        newWarpField.data
      )
      val restrictWarpFieldToDomainParmeters = restrictWarpFieldTodomain(domain, df)

      val warpedField = warper.transformWithField(domain.domain, restrictWarpFieldToDomainParmeters._1)
      DomainWithPoseParameters[_3D, DDomain](warpedField,
                                             restrictWarpFieldToDomainParmeters._2,
                                             restrictWarpFieldToDomainParmeters._3)
    }
    /** Rigidly transforms the DomainWithPoseParameters and turn the
      * DomainWithPoseParameters in new position */
    override def transform(domain: DomainWithPoseParameters[_3D, DDomain],
                           transformation: Transformation[_3D]): DomainWithPoseParameters[_3D, DDomain] = {

      DomainWithPoseParameters(
        warper.transform(domain.domain, transformation),
        transformation(domain.rotCenter),
        transformation(domain.neutralPoint)
      )
    }
  }

  implicit val domainWithPoseParametersTraiangleMesh = new WarperDomainWithPoseParameters[TriangleMesh]()
  implicit val domainWithPoseParametersTetrahedralMesh = new WarperDomainWithPoseParameters[TetrahedralMesh]()

}
