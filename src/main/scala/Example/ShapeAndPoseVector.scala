package Example
import ShapeAndPoseModels.ShapeAndPoseVector
import scalismo.geometry.{EuclideanVector, Point3D, _3D}
object ShapeAndPoseVectors {
  def main(args: Array[String]): Unit = {
    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)

    val shapeVec1: EuclideanVector[_3D]= Point3D(4.0, 5.0, 6.0).toVector
    val poseVec1: EuclideanVector[_3D]= Point3D(4.0, 5.0, 6.0).toVector

    val shapeVec2: EuclideanVector[_3D]= Point3D(4.0, 5.0, 6.0).toVector
    val poseVec2: EuclideanVector[_3D]= Point3D(4.0, 5.0, 6.0).toVector

    val v1:ShapeAndPoseVector[_3D] = ShapeAndPoseVector(shapeVec1,poseVec1)
    val v2:ShapeAndPoseVector[_3D] = ShapeAndPoseVector(shapeVec2,poseVec2)

    val v=ShapeAndPoseVector(shapeVec1-shapeVec2,poseVec1-poseVec2)

  }

  }
