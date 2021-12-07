package ShapeAndPoseModels

import breeze.linalg.DenseVector
import scalismo.common.Vectorizer
import scalismo.geometry.{_3D, EuclideanVector, EuclideanVector3D}
/**
  * An 3-dimensional Vector
  * @param shapeVec: vector of shape features
  * @param poseVec: vector of pose feature
  * creates from two [[EuclideanVector]], one for shape and the other for pose features
  */
case class ShapeAndPoseVector[D](shapeVec: EuclideanVector[D], poseVec: EuclideanVector[D])

object ShapeAndPoseVector {
  implicit object shapeAndPoseVectorVectorizer extends Vectorizer[ShapeAndPoseVector[_3D]] {
    override def dim: Int = 6

    override def vectorize(v: ShapeAndPoseVector[_3D]): DenseVector[Double] =
      DenseVector(v.shapeVec(0), v.shapeVec(1), v.shapeVec(2), v.poseVec(0), v.poseVec(1), v.poseVec(2))

    override def unvectorize(d: DenseVector[Double]): ShapeAndPoseVector[_3D] =
      ShapeAndPoseVector[_3D](EuclideanVector3D(d(0), d(1), d(2)), EuclideanVector3D(d(3), d(4), d(5)))
  }
}
