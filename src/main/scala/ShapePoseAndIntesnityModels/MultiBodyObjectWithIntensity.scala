package ShapePoseAndIntesnityModels


import ShapeAndPoseModels.MultiBodyObject.WarperMultiBodyObjects
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
case class MultiBodyObjectWithIntensity[D: NDSpace: UnstructuredPoints.Create, DDomain[D] <: DiscreteDomain[D]](
  val objects: List[DDomain[D]],
  val intensity: List[IndexedSeq[Double]],
  val rotationCenters: List[Point[D]],
  val neutralPoints: List[Point[D]]
)(implicit val warper: DomainWarp[D, DDomain])
    extends DiscreteDomain[D] {

  type DomainT[D] = MultiBodyObjectWithIntensity[D, DDomain]

  val intensityDomain=(for (i <- 0 to objects.size - 1) yield {
    intensity(i) ++ IndexedSeq(0.0) ++ IndexedSeq(0.0)
  }).reduce((acc, s) => acc ++ s)

  override def pointSet: PointSet[D] =
    UnstructuredPoints((for (i <- 0 to objects.size - 1) yield {
      objects(i).pointSet.points.toIndexedSeq ++ IndexedSeq(rotationCenters(i)) ++ IndexedSeq(neutralPoints(i))
    }).reduce((acc, s) => acc ++ s))

  /** Rigidly transforms the MultibodyObject and turn the
    * MultibodyObject in new position */
  def transform(transform: List[RigidTransformation[D]]): MultiBodyObjectWithIntensity[D, DDomain] = {

    MultiBodyObjectWithIntensity[D, DDomain](
      (for (i <- 0 to transform.size - 1) yield {
        warper.transform(objects(i), transform(i))
      }).toList,
      (for (i <- 0 to transform.size - 1) yield {
        intensity(i)
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

object MultiBodyObjectWithIntensity {

  class WarperMultiBodyObjectWithIntensity[DDomain[D] <: DiscreteDomain[D]](implicit warper: DomainWarp[_3D, DDomain])
       {

    /**
      * restricts the DiscreteField over the MultiBodyObjectWithIntensity the DiscreteField over a DDomain
      */
    def restrictWarpFieldToSubDomain(
      domain: DDomain[_3D],
      warpField: DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]]
    ): DiscreteField[_3D, DDomain, PointWithIntensityVectorVector[_3D]] = {
      val warpFieldInterpolated = warpField.interpolate(NearestNeighborInterpolator())

      val d = DiscreteField(domain, warpFieldInterpolated)
      d
    }

    /**
      * Warp the points of the MultiBodyObjectWithIntensity and turn it into the
      * warped DomainWithPoseParameters
      */
    def transformWithField(
                                     domain: MultiBodyObjectWithIntensity[_3D, DDomain],
                                     newWarpField: DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]]
    ): MultiBodyObjectWithIntensity[_3D, DDomain] = {

      val warpField =
        DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, DDomain]#DomainT, PointWithIntensityVectorVector[_3D]](domain, newWarpField.data)
      MultiBodyObjectWithIntensity(
        (for (i <- 0 to domain.objects.size - 1) yield {
          val warpFieldOnObject1 = restrictWarpFieldToSubDomain(domain.objects(i), warpField)
          val dfi=DiscreteField(domain.objects(i), warpFieldOnObject1.values.map(_.Vec).toIndexedSeq)
          warper.transformWithField(domain.objects(i), dfi)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          val warpFieldOnObject1 = restrictWarpFieldToSubDomain(domain.objects(i), warpField)
          domain.intensity(i).zipWithIndex.map(vi=>vi._1+warpFieldOnObject1.data(vi._2).Intensity)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          domain.rotationCenters(i)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          domain.neutralPoints(i)
        }).toList
      )
    }
    /** Rigidly transforms the MultiBodyObjectWithIntensity and turn the
      * MultiBodyObjectWithIntensity in new position */
    def transform(domain: MultiBodyObjectWithIntensity[_3D, DDomain],
                           transformation: Transformation[_3D]): MultiBodyObjectWithIntensity[_3D, DDomain] = {

      MultiBodyObjectWithIntensity(
        (for (i <- 0 to domain.objects.size - 1) yield {
          warper.transform(domain.objects(i), transformation)
        }).toList,
        (for (i <- 0 to domain.objects.size - 1) yield {
          domain.intensity(i)
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

  //implicit val warperMultibodyObjectsSTriangleMesh = new WarperMultiBodyObjects[TriangleMesh]()
  //implicit val warperMultibodyObjectsTetrahedarlMesh = new WarperMultiBodyObjects[TetrahedralMesh]()
}
