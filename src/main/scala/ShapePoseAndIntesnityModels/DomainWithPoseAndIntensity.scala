package ShapePoseAndIntesnityModels

import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common._
import scalismo.geometry.{NDSpace, Point, _3D}
import scalismo.transformations.{RigidTransformation, Transformation}

/** @param domain: a discreteDomain
  * @param rotCenter: a point
  * @param neutralPoint: a point
  * Defines DiscreteDomain with its associated rotation center  and neutral points.
  * the rotation center is the point that is invariant in the rotation of the DiscreteDomain
  * the neutral point is any point other than the points of the DiscreteDomain and the rotation center
  */
case class DomainWithPoseAndIntensity[D: NDSpace: UnstructuredPoints.Create, DDomain[D] <: DiscreteDomain[D]](
  domain: DDomain[D],
  intensity: IndexedSeq[Double],
  rotCenter: Point[D],
  neutralPoint: Point[D]
)(implicit val warper: DomainWarp[D, DDomain])
    extends DiscreteDomain[D] {

  type DomainT[D] = DomainWithPoseAndIntensity[D, DDomain]
  val intensityDomain=intensity ++ IndexedSeq(0.0) ++ IndexedSeq(0.0)

  override def pointSet: PointSet[D] = UnstructuredPoints(
    domain.pointSet.points.toIndexedSeq ++ IndexedSeq(rotCenter) ++ IndexedSeq(neutralPoint)
  )

  /** Rigidly transforms the DomainWithPoseAndIntensity and turn the
    * DomainWithPoseAndIntensity in new position */
  def transform(transform: RigidTransformation[D]): DomainWithPoseAndIntensity[D, DDomain] = {
    DomainWithPoseAndIntensity[D, DDomain](
      warper.transform(domain, transform),
      intensity,
      transform.apply(rotCenter),
      transform.apply(neutralPoint)
    )
  }

}

object DomainWithPoseAndIntensity {

  class WarperDomainWithPoseAndIntensity[DDomain[D] <: DiscreteDomain[D]](implicit warper: DomainWarp[_3D, DDomain])
  {

    /**
      * restricts the DiscreteField over the DomainWithPoseAndIntensity the DiscreteField over a DDomain
      */
    def restrictWarpFieldToSubDomain(
                                      domain: DDomain[_3D],
                                      warpField: DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]]
                                    ): DiscreteField[_3D, DDomain, PointWithIntensityVectorVector[_3D]] = {
      val warpFieldInterpolated = warpField.interpolate(NearestNeighborInterpolator())

      val d = DiscreteField(domain, warpFieldInterpolated)
      d
    }

    /**
      * Warp the points of the DomainWithPoseAndIntensity and turn it into the
      * warped DomainWithPoseParameters
      */
    def transformWithField(
                            domain: DomainWithPoseAndIntensity[_3D, DDomain],
                            newWarpField: DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]]
                          ): DomainWithPoseAndIntensity[_3D, DDomain] = {

      val warpField =
        DiscreteField[_3D, DomainWithPoseAndIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]](domain, newWarpField.data)
      val warpFieldOnObject1 = restrictWarpFieldToSubDomain(domain.domain, warpField)
      val dfi=DiscreteField(domain.domain, warpFieldOnObject1.values.map(_.Vec).toIndexedSeq)

      val warpFieldOnObject2 = restrictWarpFieldToSubDomain(domain.domain, warpField)

      DomainWithPoseAndIntensity(
          warper.transformWithField(domain.domain, dfi),
          domain.intensity.zipWithIndex.map(vi=>vi._1+warpFieldOnObject1.data(vi._2).Intensity),
          domain.rotCenter,
          domain.neutralPoint,
      )
    }
    /** Rigidly transforms the DomainWithPoseAndIntensity and turn the
      * DomainWithPoseAndIntensity in new position */
    def transform(domain: DomainWithPoseAndIntensity[_3D, DDomain],
                  transformation: Transformation[_3D]): DomainWithPoseAndIntensity[_3D, DDomain] = {

      DomainWithPoseAndIntensity(
          warper.transform(domain.domain, transformation),
          domain.intensity,
          transformation(domain.rotCenter),
          transformation(domain.neutralPoint)
      )

    }
  }

}
