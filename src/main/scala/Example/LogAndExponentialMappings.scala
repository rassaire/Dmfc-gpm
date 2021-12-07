package Example
import java.io.File

import ShapeAndPoseModels.{DomainWithPoseParameters, ShapeAndPoseVector, SinglePoseExpLogMapping}
import scalismo.common.DiscreteField
import scalismo.geometry.{EuclideanVector, Landmark, _3D}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.Transformation
object LogAndExponentialMappings {

  def main(args: Array[String]): Unit = {
    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)
    //Let's start by calling the DomainWithPoseparamter from [tutorial 1](tutorial1) again, which we now call targetDomainWithPoseParam.
    val triangleMesh = MeshIO.readMesh(new File("path/trianglemesh.stl")).get
    val RotCent = LandmarkIO.readLandmarksJson[_3D](new File("path/rotcenter.json")).get

    val targetDomainWithPoseParam=DomainWithPoseParameters(triangleMesh,
      RotCent.head.point,
      RotCent.head.point
    )
    //Let's laod another DomainWithPOseparamter, which we call referenceDomainWithPoseParam
    val referenceMesh:TriangleMesh[_3D] = MeshIO.readMesh(new File("path/referencemesh.stl")).get
    val referenceRotCent:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson[_3D](new File("path/refrotcenter.json")).get

    val referenceDomainWithPoseParam: DomainWithPoseParameters[_3D, TriangleMesh]=DomainWithPoseParameters(referenceMesh,
      referenceRotCent.head.point,
      referenceRotCent.head.point
    )
    //Define the log and expo mapping
    val logExp:SinglePoseExpLogMapping[TriangleMesh]=SinglePoseExpLogMapping(referenceDomainWithPoseParam)

    //Let's now compute the [defromation field](https://scalismo.org/docs/tutorials/tutorial3) represenating targetDomainWithPoseParam with respect to referenceDomainWithPoseParam.
    val df=DiscreteField[_3D, ({ type T[_3D] = DomainWithPoseParameters[_3D, TriangleMesh] })#T, EuclideanVector[_3D]](referenceDomainWithPoseParam,
      referenceDomainWithPoseParam.pointSet.pointsWithId.toIndexedSeq.map(pt =>targetDomainWithPoseParam.pointSet.point(pt._2) - pt._1))

    val logDF: DiscreteField[_3D, ({type T[D] = DomainWithPoseParameters[D, TriangleMesh]})#T, ShapeAndPoseVector[_3D]]=logExp.logMappingSingleDomain(df)

    //Computation of the exp function of targetDomainWithPoseParam, which is a rigid transformation.
    // The exp function projects the pose defromation field component from the vector space into the manifold.
    val expDF: Transformation[_3D] = logExp.expMappingSingleDomain(df)


  }
}
