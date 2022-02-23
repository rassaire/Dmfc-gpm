package FittingWithMCMC

import ShapeAndPoseModels._
import ShapePoseAndIntesnityModels._
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteField, DomainWarp}
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.mesh.{TetrahedralMesh, TriangleMesh}
import scalismo.sampling.loggers.AcceptRejectLogger
import scalismo.sampling.{DistributionEvaluator, ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.{LowRankGaussianProcess, MultivariateNormalDistribution}
import scalismo.transformations.{RigidTransformation, Rotation3D, Translation3D, TranslationAfterRotation3D}
import scalismo.utils.Memoize
import scalismo.utils.Random.implicits._


case class Parameters(modelCoefficients: DenseVector[Double],
                      translationParameters1: EuclideanVector[_3D],
                      rotationParameters1: (Double, Double, Double),
                      translationParameters2: EuclideanVector[_3D],
                      rotationParameters2: (Double, Double, Double))




case class Sample(generatedBy : String, parameters : Parameters,rotationCenter1: Point[_3D],
                  rotationCenter2: Point[_3D]) {
  def poseTransformation : (RigidTransformation[_3D],RigidTransformation[_3D]) = {

    val translation1 = Translation3D(parameters.translationParameters1)
    val translation2 = Translation3D(parameters.translationParameters2)
    val rotation1 = Rotation3D(
      parameters.rotationParameters1._1,
      parameters.rotationParameters1._2,
      parameters.rotationParameters1._3,
      rotationCenter1
    )
    val rotation2 = Rotation3D(
      parameters.rotationParameters2._1,
      parameters.rotationParameters2._2,
      parameters.rotationParameters2._3,
      rotationCenter2
    )

    (TranslationAfterRotation3D(translation1, rotation1),
      TranslationAfterRotation3D(translation2, rotation2))
  }
}

//
//case class PriorEvaluator(model: MultiBodyShapeAndPosePGA[TetrahedralMesh])
//  extends DistributionEvaluator[Sample] {
//
//
//  override def logValue(sample: Sample): Double = {
//    model.gp.logpdf(sample.parameters.modelCoefficients)
//
//  }
//
//}

case class PriorEvaluatorMultiShapePoseModel(model: MultiBodyShapeAndPosePGA[TetrahedralMesh])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    model.gp.logpdf(sample.parameters.modelCoefficients)

  }

}

case class PriorEvaluatorMultiShapePoseAndIntensityModel(model: MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    model.gp.logpdf(sample.parameters.modelCoefficients)

  }

}

//case class ProximityEvaluator(model:ShapeAndPosePDM[TriangleMesh] ,
//                              targetpoint:Seq[(Point[_3D],MultivariateNormalDistribution)])
//  extends DistributionEvaluator[Sample] {
//
//
//  override def logValue(sample: Sample): Double = {
//
//val currModelInstance = model.instance(sample.parameters.modelCoefficients).domain
//
//val likelihoods = targetpoint.map(targetLandmark =>{
//val (targetlandmarkpoint, uncertainty) = targetLandmark
//val closestPointCurrentFit = currModelInstance.pointSet.findClosestPoint(targetlandmarkpoint).point
//val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
//uncertainty.logpdf(observedDeformation.toBreezeVector)
//})
//
//likelihoods.sum
//}
//
//}


case class ProximityEvaluatorMultiShapePoseModel(model:MultiBodyShapeAndPosePGA[TriangleMesh] ,
                                       targetPoint:List[Seq[(Point[_3D],MultivariateNormalDistribution)]])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    val currModelInstance = model.instance(sample.parameters.modelCoefficients).objects
    val likelihoods=(for (i<-0 to model.gp._domain.objects.size-1) yield {
      targetPoint(i).map(targetLandmark => {
        val (targetlandmarkpoint, uncertainty) = targetLandmark
        val closestPointCurrentFit = currModelInstance(i).pointSet.findClosestPoint(targetlandmarkpoint).point
        val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
        uncertainty.logpdf(observedDeformation.toBreezeVector)
      })
    }).flatten
    likelihoods.sum
  }

}

case class ProximityEvaluatorMultiShapePoseAndIntensityModel(model:MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh] ,
                              targetPoint:List[Seq[(Point[_3D],MultivariateNormalDistribution)]])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    val currModelInstance = model.instance(sample.parameters.modelCoefficients).objects.map(m=>m.operations.getOuterSurface)
    val likelihoods=(for (i<-0 to model.gp._domain.objects.size-1) yield {
      targetPoint(i).map(targetLandmark => {
        val (targetlandmarkpoint, uncertainty) = targetLandmark
        val closestPointCurrentFit = currModelInstance(i).pointSet.findClosestPoint(targetlandmarkpoint).point
        val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
        uncertainty.logpdf(observedDeformation.toBreezeVector)
      })
    }).flatten
    likelihoods.sum
  }

}

case class ProximityEvaluatorMultiShapePoseAndIntensitySingleObservation(model:MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh] ,
                                       targetpoint:Seq[(Point[_3D],MultivariateNormalDistribution)])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    val t1 = System.nanoTime
    val currModelInstance = Util.mergeMeshes(model.instance(sample.parameters.modelCoefficients).objects.map(m=>m.operations.getOuterSurface))


    val likelihoods = targetpoint.map(targetLandmark =>{
      val (targetlandmarkpoint, uncertainty) = targetLandmark
      val closestPointCurrentFit = currModelInstance.pointSet.findClosestPoint(targetlandmarkpoint).point
      val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
      uncertainty.logpdf(observedDeformation.toBreezeVector)
    })
    likelihoods.sum
  }

}


case class ProximityEvaluatorMultiShapePoseModelSingleObservation(model:MultiBodyShapeAndPosePGA[TriangleMesh] ,
                                                        targetpoint:Seq[(Point[_3D],MultivariateNormalDistribution)])
  extends DistributionEvaluator[Sample] {


  override def logValue(sample: Sample): Double = {
    val t1 = System.nanoTime
    val currModelInstance = Util.mergeMeshes(model.instance(sample.parameters.modelCoefficients).objects)


    val likelihoods = targetpoint.map(targetLandmark =>{
      val (targetlandmarkpoint, uncertainty) = targetLandmark
      val closestPointCurrentFit = currModelInstance.pointSet.findClosestPoint(targetlandmarkpoint).point
      val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
      uncertainty.logpdf(observedDeformation.toBreezeVector)
    })
    likelihoods.sum
  }

}


//case class ProximityEvaluatorContinuousMultiBody(model: LowRankGaussianProcess[_3D, ShapeAndPoseVector[_3D]],
//                                                 logExp:PoseExpLogMapping[TriangleMesh],
//                                                 reference:MultibodyObject[_3D, TriangleMesh],
//                                      targetpoint:List[Seq[(Point[_3D],MultivariateNormalDistribution)]])(implicit warper: DomainWarp[_3D, TriangleMesh])
//  extends DistributionEvaluator[Sample] {
//
//
//  override def logValue(sample: Sample): Double = {
//
//    val currModelInstance = model.instance(sample.parameters.modelCoefficients)
//    val currModelInstanceDiscrete = DiscreteField(reference, currModelInstance)
//
//
//    val shapeField = DiscreteField[_3D, MultibodyObject[_3D, TriangleMesh]#DomainT, EuclideanVector[_3D]](
//      reference,
//      currModelInstanceDiscrete.values.map(_.shapeVec).toIndexedSeq
//    ).interpolate(NearestNeighborInterpolator())
//
//
//    val poseField = DiscreteField[_3D, MultibodyObject[_3D, TriangleMesh]#DomainT, EuclideanVector[_3D]](
//      reference,
//      currModelInstanceDiscrete.values.map(_.poseVec).toIndexedSeq
//    )
//    val t1 = System.nanoTime
//    val poseTransforms = logExp.expMapping(poseField)
//    val duration = (System.nanoTime - t1) *1e-9
//    println("coeffs running time "+duration+" seconds")
//
//    val shapes= for (i<-0 to reference.objects.size-1) yield {
//      warper.transformWithField(reference.objects(i),DiscreteField(reference.objects(i),shapeField)).transform(poseTransforms(i))
//    }
//
//
//
//    val likelihoods=(for (i<-0 to reference.objects.size-1) yield {
//      targetpoint(i).map(targetLandmark => {
//        val (targetlandmarkpoint, uncertainty) = targetLandmark
//        val closestPointCurrentFit = shapes(i).pointSet.findClosestPoint(targetlandmarkpoint).point
//        val observedDeformation = targetlandmarkpoint - closestPointCurrentFit
//        uncertainty.logpdf(observedDeformation.toBreezeVector)
//      })
//    }).flatten
//
//    likelihoods.sum
//  }
//
//}


case class CachedEvaluator[A](evaluator: DistributionEvaluator[A]) extends DistributionEvaluator[A] {
  val memoizedLogValue = Memoize(evaluator.logValue, 10)

  override def logValue(sample: A): Double = {
    memoizedLogValue(sample)
  }
}

case class ShapeUpdateProposal(paramVectorSize : Int, stddev: Double)
  extends ProposalGenerator[Sample]  with TransitionProbability[Sample] {

  val perturbationDistr = new MultivariateNormalDistribution(
    DenseVector.zeros(paramVectorSize),
    DenseMatrix.eye[Double](paramVectorSize) * stddev * stddev
  )


  override def propose(sample: Sample): Sample = {
    val perturbation = perturbationDistr.sample()
    val newParameters = sample.parameters.copy(modelCoefficients = sample.parameters.modelCoefficients + perturbationDistr.sample)
    sample.copy(generatedBy = s"ShapeUpdateProposal ($stddev)", parameters = newParameters)
  }

  override def logTransitionProbability(from: Sample, to: Sample) = {
    val residual = to.parameters.modelCoefficients - from.parameters.modelCoefficients
    perturbationDistr.logpdf(residual)
  }
}


case class RotationUpdateProposal(stddev: Double) extends
  ProposalGenerator[Sample]  with TransitionProbability[Sample] {
  val perturbationDistr = new MultivariateNormalDistribution(
    DenseVector.zeros[Double](3),
    DenseMatrix.eye[Double](3) * stddev * stddev)

  def propose(sample: Sample): Sample= {
    val perturbation = perturbationDistr.sample

    val newRotationParameters1 = (
      sample.parameters.rotationParameters1._1 + perturbation(0),
      sample.parameters.rotationParameters1._2 + perturbation(1),
      sample.parameters.rotationParameters1._3 + perturbation(2),
    )

    val newRotationParameters2 = (
      sample.parameters.rotationParameters2._1 + perturbation(0),
      sample.parameters.rotationParameters2._2 + perturbation(1),
      sample.parameters.rotationParameters2._3 + perturbation(2)
    )

    val newParameters = sample.parameters.copy(rotationParameters1 = newRotationParameters1,
      rotationParameters2 = newRotationParameters2)
    sample.copy(generatedBy = s"RotationUpdateProposal1 ($stddev)", parameters = newParameters)

  }
  override def logTransitionProbability(from: Sample, to: Sample) = {
    val residual1 = DenseVector(
      to.parameters.rotationParameters1._1 - from.parameters.rotationParameters1._1,
      to.parameters.rotationParameters1._2 - from.parameters.rotationParameters1._2,
      to.parameters.rotationParameters1._3 - from.parameters.rotationParameters1._3
    )

    val residual2 = DenseVector(
      to.parameters.rotationParameters2._1 - from.parameters.rotationParameters2._1,
      to.parameters.rotationParameters2._2 - from.parameters.rotationParameters2._2,
      to.parameters.rotationParameters2._3 - from.parameters.rotationParameters2._3
    )

    perturbationDistr.logpdf((residual1+residual2)*(1.0/3))

  }
}


case class TranslationUpdateProposal(stddev: Double) extends
  ProposalGenerator[Sample]  with TransitionProbability[Sample] {

  val perturbationDistr = new MultivariateNormalDistribution( DenseVector.zeros(3),
    DenseMatrix.eye[Double](3) * stddev * stddev)

  def propose(sample: Sample): Sample= {
    val newTranslationParameters1 = sample.parameters.translationParameters1 + EuclideanVector.fromBreezeVector(perturbationDistr.sample())
    val newTranslationParameters2 = sample.parameters.translationParameters2 + EuclideanVector.fromBreezeVector(perturbationDistr.sample())
    val newParameters = sample.parameters.copy(translationParameters1 = newTranslationParameters1,
      translationParameters2 = newTranslationParameters2)
    sample.copy(generatedBy = s"TranlationUpdateProposal1 ($stddev)", parameters = newParameters)

  }

  override def logTransitionProbability(from: Sample, to: Sample) = {

    val residual = to.parameters.translationParameters1 - from.parameters.translationParameters1
    val residua2 = to.parameters.translationParameters2 - from.parameters.translationParameters2
    perturbationDistr.logpdf(((residual+residua2*(1.0/3)).toBreezeVector))

  }
}


class MixtureLogger extends AcceptRejectLogger[Sample] {
  private val numAccepted = collection.mutable.Map[String, Int]()
  private val numRejected = collection.mutable.Map[String, Int]()

  override def accept(current: Sample,
                      sample: Sample,
                      generator: ProposalGenerator[Sample],
                      evaluator: DistributionEvaluator[Sample],


                     ): Unit = {
    val numAcceptedSoFar = numAccepted.getOrElseUpdate(sample.generatedBy, 0)
    numAccepted.update(sample.generatedBy, numAcceptedSoFar + 1)
  }

  override def reject(current: Sample,
                      sample: Sample,
                      generator: ProposalGenerator[Sample],
                      evaluator: DistributionEvaluator[Sample],
                     ): Unit = {
    val numRejectedSoFar = numRejected.getOrElseUpdate(sample.generatedBy, 0)
    numRejected.update(sample.generatedBy, numRejectedSoFar + 1)
  }


  def acceptanceRatios() : Map[String, Double] = {
    val generatorNames = numRejected.keys.toSet.union(numAccepted.keys.toSet)
    val acceptanceRatios = for (generatorName <- generatorNames ) yield {
      val total = (numAccepted.getOrElse(generatorName, 0)
        + numRejected.getOrElse(generatorName, 0)).toDouble
      (generatorName, numAccepted.getOrElse(generatorName, 0) / total)
    }
    acceptanceRatios.toMap
  }
}
