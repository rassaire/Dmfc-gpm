package ShapePoseAndIntesnityModels

import breeze.linalg.DenseVector
import scalismo.common.Vectorizer
import scalismo.geometry.{_3D, EuclideanVector, EuclideanVector3D}
/**
  * An 3-dimensional Vector
  * @param shapeVec: vector of shape features
  * @param poseVec: vector of pose feature
  * creates from two [[EuclideanVector]], one for shape and the other for pose features
  */


case class PointWithIntensityVectorVector[D](Vec: EuclideanVector[D], Intensity: Double)

object  PointWithIntensityVectorVector {
  implicit object  PointWithIntensityVectorVectorVectorizer extends Vectorizer[ PointWithIntensityVectorVector[_3D]] {
    override def dim: Int = 4

    override def vectorize(v:  PointWithIntensityVectorVector[_3D]): DenseVector[Double] =
      DenseVector(v.Vec(0), v.Vec(1), v.Vec(2), v.Intensity)

    override def unvectorize(d: DenseVector[Double]):  PointWithIntensityVectorVector[_3D] =
      PointWithIntensityVectorVector[_3D](EuclideanVector3D(d(0), d(1), d(2)), d(3))
  }
}

case class ShapePoseAndIntensityVector[D](shapeAndIntensityVec: PointWithIntensityVectorVector[_3D], poseVec: EuclideanVector[D])
object ShapePoseAndIntensityVector {
  implicit object ShapePoseAndIntensityVectorVectorizer extends Vectorizer[ShapePoseAndIntensityVector[_3D]] {
    override def dim: Int = 7

    override def vectorize(v: ShapePoseAndIntensityVector[_3D]): DenseVector[Double] =
      DenseVector(v.shapeAndIntensityVec.Vec(0), v.shapeAndIntensityVec.Vec(1), v.shapeAndIntensityVec.Vec(2),
        v.shapeAndIntensityVec.Intensity, v.poseVec(0), v.poseVec(1), v.poseVec(2))

    override def unvectorize(d: DenseVector[Double]): ShapePoseAndIntensityVector[_3D] =
      ShapePoseAndIntensityVector[_3D](PointWithIntensityVectorVector[_3D](EuclideanVector3D(d(0), d(1), d(2)),d(3)), EuclideanVector3D(d(4), d(5), d(6)))
  }
}

//case class ShapeAndPoseVector[D](shapeVec: EuclideanVector[D], poseVec: EuclideanVector[D])
//
//object ShapeAndPoseVector {
//  implicit object shapeAndPoseVectorVectorizer extends Vectorizer[ShapeAndPoseVector[_3D]] {
//    override def dim: Int = 6
//
//    override def vectorize(v: ShapeAndPoseVector[_3D]): DenseVector[Double] =
//      DenseVector(v.shapeVec(0), v.shapeVec(1), v.shapeVec(2), v.poseVec(0), v.poseVec(1), v.poseVec(2))
//
//    override def unvectorize(d: DenseVector[Double]): ShapeAndPoseVector[_3D] =
//      ShapeAndPoseVector[_3D](EuclideanVector3D(d(0), d(1), d(2)), EuclideanVector3D(d(3), d(4), d(5)))
//  }
//}
