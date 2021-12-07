package Example
import scalismo.io.{LandmarkIO, MeshIO}
import java.io.File
import ShapeAndPoseModels.DomainWithPoseParameters
import scalismo.geometry._3D
import ShapeAndPoseModels.MultiBodyObject
import scalismo.mesh.TriangleMesh
object DataStructures {


  def main(args: Array[String]): Unit = {
    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)


    val TriangleMesh = MeshIO.readMesh(new File("path/trianglemesh.stl")).get
    val TetrahedralMesh = MeshIO.readTetrahedralMesh(new File("path/tetrahedralmesh.stl")).get

    val RotCent = LandmarkIO.readLandmarksJson[_3D](new File("path/rotcenter.json")).get
    val DomainWithPoseParam=DomainWithPoseParameters(TriangleMesh,
      RotCent.head.point,
      RotCent.head.point
    )

    val TriangleMesh1 = MeshIO.readMesh(new File("path/trianglemesh1.stl")).get
    val TriangleMesh2 = MeshIO.readMesh(new File("path/trianglemesh2.stl")).get

    val rotCent1 = LandmarkIO.readLandmarksJson[_3D](new File("path/rotcenter1.json")).get
    val rotCent2 = LandmarkIO.readLandmarksJson[_3D](new File("path/rotcenter2.json")).get


    val multibodyObject:MultiBodyObject[_3D, TriangleMesh] = MultiBodyObject(
      List(TriangleMesh1,TriangleMesh2),
      List(rotCent1.head.point,rotCent2.head.point),
      List(rotCent1.head.point,rotCent2.head.point)

    )


  }
}
