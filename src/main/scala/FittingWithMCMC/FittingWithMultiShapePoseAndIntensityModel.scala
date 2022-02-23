package FittingWithMCMC

import java.awt.Color
import java.io.File

import scalismo.common.{DiscreteDomain, DiscreteField, DiscreteField3D, ScalarMeshField}
import ShapePoseAndIntesnityModels._
import ShapePoseAndIntesnityModels.io.MultibodyPGAIO
import scalismo.common
import scalismo.common.interpolation.NearestNeighborInterpolator3D
import scalismo.mesh.TetrahedralMesh
//import Application.fitting.runMCMC
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.{MeshMetrics, TriangleMesh}
import scalismo.sampling.TransitionProbability
import scalismo.sampling.algorithms.MetropolisHastings
import scalismo.sampling.evaluators.ProductEvaluator
import scalismo.sampling.proposals.MixtureProposal
import scalismo.statisticalmodel.MultivariateNormalDistribution
import scalismo.ui.api.ScalismoUI

object FittingWithMultiShapePoseAndIntensityModel {

  def main (args: Array[String]): Unit= {

    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)
    val ui = ScalismoUI()
    val t1 = System.nanoTime

    val num="020"
    //read your  model
    println("reading model ...")
    val ShapePoseAndIntensityPGA = MultibodyPGAIO.readShapeAndPoseMultiBodyWithIntensityPGA(new File("E:\\PostDoc_Inserm\\vae_project\\knee_data_for_dmfc\\models\\kneeDmfc.h5")).get
  println("ended reading model!")

    //read surface meshes (target) to you want to fit the model onto
    val targetMesh1: TriangleMesh[_3D] = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\partial_knees\\"+num+"_2_femur\\femurAlignedToModel.stl")).get//.transform(t)

    val targetMesh2: TriangleMesh[_3D] = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\partial_knees\\"+num+"_2_tibia\\tibiaAlignedToModel.stl")).get//.transform(t)


    // read your ground truth, this can just be the combined of target mesh into single mesh
    val groundTruth = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\ground_truth\\"+num+"_2_KneeAlignedToModel.stl")).get
    //val ShapeAndPosePGA = MultibodyPDMIO.readShapeAndPoseMultiBodyPDM(new File("E:\\PostDoc_Inserm\\knee data\\data in correspondence\\knee model sampling\\shapePoseModelPGAKnee.h5")).get
    //TestModels.visualizingSSM(ShapeAndPosePGA.shapePDM,10)

    val targetGroup = ui.createGroup("target meshes")
    val targetView1 = ui.show(targetGroup, targetMesh1, "target mesh")
    val targetView2 = ui.show(targetGroup, targetMesh2, "target mesh")
    targetView1.color = Color.blue
    targetView2.color = Color.blue

    println("here")
    val landmarkNoiseVariance = 1.0
    val uncertainty = MultivariateNormalDistribution(
      DenseVector.zeros[Double](3),
      DenseMatrix.eye[Double](3) * landmarkNoiseVariance
    )
//    val sample1=UniformMeshSampler3D(targetMesh1, targetMesh1.pointSet.numberOfPoints)
//    val sample2=UniformMeshSampler3D(targetMesh2, targetMesh1.pointSet.numberOfPoints)

    //val targetPoints: Seq[(Point[_3D],MultivariateNormalDistribution)] = targetMesh1.pointSet.pointsWithId.map { pi => (pi._1,uncertainty )}.toSeq
    val interpolatedGP =NearestNeighborInterpolator3D[({ type λ[D] = MultiBodyObjectWithIntensity[D, TetrahedralMesh] })#λ,  ShapePoseAndIntensityVector[_3D]]()
    //val int=


    val targetPoints: List[Seq[(Point[_3D], MultivariateNormalDistribution)]] = List(targetMesh1,targetMesh2).map(targetMesh=>targetMesh.pointSet.pointsWithId.map { pi => (pi._1,uncertainty) }.toSeq)
   // val model=ShapeAndPosePGA.gp.interpolate(interpolatedGP)//.truncate(20)

    //val likelihoodEvaluator = CachedEvaluator(MCMC.ProximityEvaluatorMultiBodySingleObservation(ShapeAndPosePGA, targetPoints))
    //implicit val rand = scalismo.utils.Random(42)
//    val distrDim1 = Uniform(0, targetPoints.size)(rng.breezeRandBasis)
//    val samplePoints =(0 until 2000).map(i => targetPoints(distrDim1.draw().toInt)).toList
    val likelihoodEvaluator = CachedEvaluator(ProximityEvaluatorMultiShapePoseAndIntensityModel(ShapePoseAndIntensityPGA, targetPoints))
//    val likelihoodEvaluator = CachedEvaluator(MCMC.ProximityEvaluatorContinuousMultiBody(model,
//      ShapeAndPosePGA.logExpMapping,
//      ShapeAndPosePGA.gp.domain,
//      targetPoints))


    val shapeUpdateProposalLargeStep = ShapeUpdateProposal(ShapePoseAndIntensityPGA.gp.rank, 0.3)
   // val shapeUpdateProposalSmallStep=ShapeUpdateProposal(ShapeAndPosePGA.gp.rank, 0.1)

    val generator = MixtureProposal.fromProposalsWithTransition(
      (1.0, shapeUpdateProposalLargeStep))//,
     // (0.3, shapeUpdateProposalSmallStep))


    val priorEvaluator = CachedEvaluator(PriorEvaluatorMultiShapePoseAndIntensityModel(ShapePoseAndIntensityPGA))

    val posteriorEvaluator = ProductEvaluator(priorEvaluator, likelihoodEvaluator)

    runMCMC(ShapePoseAndIntensityPGA, generator, posteriorEvaluator, ui, groundTruth)
    val duration = (System.nanoTime - t1) *1.66667e-11
    println("running time "+duration)
  }




  def runMCMC(model:MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh],
              generator:MixtureProposal[Sample] with TransitionProbability[Sample],
              evaluators:ProductEvaluator[Sample],
              ui:ScalismoUI,
              targets:TriangleMesh[_3D]
             )={
    implicit val rng = scalismo.utils.Random(42)
//    val initialParameters = Parameters(
//      DenseVector.zeros[Double](model.gp.rank)
//    )

    val initialParameters = Parameters(
      DenseVector.zeros[Double](model.gp.rank),
      EuclideanVector(0, 0, 0),
      (0.0, 0.0, 0.0),
      EuclideanVector(0, 0, 0),
      (0.0, 0.0, 0.0)
    )

    //val initialSample = Sample("initial", initialParameters)

    val initialSample = Sample("initial", initialParameters,model.mean.rotationCenters(0),model.mean.rotationCenters(1))

    val chain = MetropolisHastings(generator, evaluators)
    val logger = new MixtureLogger()

    val mhIterator = chain.iterator(initialSample, logger)

    // val seq10k = mhIterator.take(600).toIndexedSeq



    val samplingIterator = for ((sample, iteration) <- mhIterator.zipWithIndex) yield {
      if (iteration % 100 == 0) {

        //visualization of the sample after 100 iterations, it is sufficient to extract the mesh surface to visualize it in order to avoid the Ui crashing when dealing  with a lot mesh volumes being displayed
        ui.show(Util.mergeMeshes(model.instance(sample.parameters.modelCoefficients).objects.map(m=>m.operations.getOuterSurface)), "sample object")
        //        ui.show(model.instance(sample.parameters.modelCoefficients).objects(1), "sample object 2")
        //        ui.show(model.instance(sample.parameters.modelCoefficients).objects(2), "sample object 3")
      }
      println("iteration " + iteration)
      sample
    }
//    val mesh=DomainWithPoseParameters(model.gp.domain.objects(0),
//      model.gp.domain.rotationCenters(0),
//      model.gp.domain.neutralPoints(0)
//    )
//    val newmodel=model.transitionToSingleObject(mesh,SinglePoseExpLogMapping(mesh)).shapePDM
//    val newinitialSample = Sample("initial", initialParameters)
//
//
//    val newchain=chain.iterator(newinitialSample, logger)

    val samples = samplingIterator.slice(800, 3500).toIndexedSeq
    println(logger.acceptanceRatios())



    val bestSample = samples.maxBy(evaluators.logValue)

    val bestmesh= for (i<-0 to model.gp.domain.objects.size) yield {
      val instance =model.instance(bestSample.parameters.modelCoefficients)

      DiscreteField3D(instance.objects(i), instance.intensity(i))

    }





    //save relevantLands as csv files , for different regions of the femur
    //saveRelevantLandList(refRelevantLand,model.gp.domain.objects(0),bestmesh(0))

    val combinedMesh = Util.mergeScalarVolumes(bestmesh.toList)

    ui.show(combinedMesh,"best fit")

    //write meshes to a file as individual oor combined
    MeshIO.writeScalarVolumeMeshField(combinedMesh,new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\reconstructedKnee.stl"))
    MeshIO.writeScalarVolumeMeshField(bestmesh.head,new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\reconstructedFemur.stl"))
    MeshIO.writeScalarVolumeMeshField(bestmesh(1),new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\reconstructedTibia.stl"))



    val dist=MeshMetrics.avgDistance(targets, combinedMesh.domain.operations.getOuterSurface)

    val errorMesh = ScalarMeshField[Double](combinedMesh.domain.operations.getOuterSurface, combinedMesh.domain.operations.getOuterSurface.pointSet.points.toIndexedSeq.map{p=>
      val closestP=targets.pointSet.findClosestPoint(p).point
      (closestP-p).norm})

    //write error as vtk scalar mesh
    MeshIO.writeScalarMeshField[Double](errorMesh, new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\error_meshes\\errorMesh.vtk"))

    // print average error
    println(dist)


  }

  /**
    * Returns the Hausdorff distance between the two meshes
    */
  def hausdorffDistance(m1: TriangleMesh[_3D], m2: TriangleMesh[_3D]): Double = {
    def allDistsBetweenMeshes(mm1: TriangleMesh[_3D], mm2: TriangleMesh[_3D]): Iterator[Double] = {
      for (ptM1 <- mm1.pointSet.points) yield {
        val cpM2 = mm2.operations.closestPointOnSurface(ptM1).point
        (ptM1 - cpM2).norm
      }
    }

    val d1 = allDistsBetweenMeshes(m1, m2)

   d1.max

  }

  def readRelevantLandList():List[Seq[Landmark[_3D]]]={
    val medialEpicondyle:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialEpicondyle.json")).get
    val lateralEpicondyle:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralEpicondyle.json")).get
    val femoralHead:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\femoralHead.json")).get
    val distalShaft:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\distalShaft.json")).get
   // val apPoint:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\apPoint.json")).get
    val anteriorIntercondyleGroove:Seq[Landmark[_3D]] =LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\anteriorIntercondyleGroove.json")).get
    val medialCondylePosterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondylePosterior.json")).get
    val lateralCondylePosterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondylePosterior.json")).get
    val medialCondyleAnterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondyleAnterior.json")).get
    val lateralCondyleAnterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondyleAnterior.json")).get
    val medialCondyleFull:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondyleFull.json")).get
    val lateralCondyleFull:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondyleFull.json")).get
    val medialSphere:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialSphere.json")).get
    val lateralSphere:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralSphere.json")).get
    val whiteSideLine:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\whiteSideLine.json")).get
    val mostAntPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostAntPointOnLateralCondyle.json")).get
    val mostAntPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostAntPointOnMedialCondyle.json")).get
    val mostDistPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostDistPointOnLateralCondyle.json")).get
    val mostDistPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostDistPointOnMedialCondyle.json")).get
    val mostPostPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostPostPointOnLateralCondyle.json")).get
    val mostPostPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostPostPointOnMedialCondyle.json")).get

    List(medialEpicondyle,
      lateralEpicondyle,
      femoralHead,
      distalShaft,
     // apPoint,
      anteriorIntercondyleGroove,
      medialCondylePosterior,
      lateralCondylePosterior,
      medialCondyleAnterior,
      lateralCondyleAnterior,
      medialCondyleFull,
      lateralCondyleFull,
      medialSphere,
      lateralSphere,
      whiteSideLine,
      mostAntPointOnLateralCondyle,
      mostAntPointOnMedialCondyle,
      mostDistPointOnLateralCondyle,
      mostDistPointOnMedialCondyle,
      mostPostPointOnLateralCondyle,
      mostPostPointOnMedialCondyle
    )
  }

  def saveRelevantLandList(relevantLand1:relevantLand, refmesh:TriangleMesh[_3D] ,bestFit:TriangleMesh[_3D]): Unit ={



    val relevantland=relevantLand(

      relevantLand1.medialEpicondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.lateralEpicondyle.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.femoralHead.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.distalShaft.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

//      relevantLand1.apPoint.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.anteriorInterCondyleGroove.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.medialCondylePosterior.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.lateralCondylePosterior.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.medialCondyleAnterior.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.lateralCondyleAnterior.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.medialCondyleFull.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.lateralCondyleFull.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.medialSphere.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.lateralSphere.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.whiteSideLine.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.mostAntPointOnLateralCondyle.map{
        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

    relevantLand1.mostAntPointOnMedialCondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.mostDistPointOnLateralCondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.mostDistPointOnMedialCondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.mostPostPointOnLateralCondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},

      relevantLand1.mostPostPointOnMedialCondyle.map{
      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))}
    )



    LandmarkIO.writeLandmarksJson(relevantland.anteriorInterCondyleGroove,new File("relevant_feature_predicted/anteriorIntercondyleGroove.json"))
   // LandmarkIO.writeLandmarksJson(relevantland.apPoint,new File("relevant_feature_predicted/apPoint.json"))
    LandmarkIO.writeLandmarksJson(relevantland.distalShaft,new File("relevant_feature_predicted/distalShaft.json"))
    LandmarkIO.writeLandmarksJson(relevantland.femoralHead,new File("relevant_feature_predicted/femoralHead.json"))
    LandmarkIO.writeLandmarksJson(relevantland.lateralCondyleAnterior,new File("relevant_feature_predicted/lateralCondyleAnterior.json"))
    LandmarkIO.writeLandmarksJson(relevantland.lateralCondyleFull,new File("relevant_feature_predicted/lateralCondyleFull.json"))
    LandmarkIO.writeLandmarksJson(relevantland.lateralCondylePosterior,new File("relevant_feature_predicted/lateralCondylePosterior.json"))
    LandmarkIO.writeLandmarksJson(relevantland.lateralEpicondyle,new File("relevant_feature_predicted/lateralEpicondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.lateralSphere,new File("relevant_feature_predicted/lateralSphere.json"))
    LandmarkIO.writeLandmarksJson(relevantland.medialCondyleAnterior,new File("relevant_feature_predicted/medialCondyleAnterior.json"))
    LandmarkIO.writeLandmarksJson(relevantland.medialCondyleFull,new File("relevant_feature_predicted/medialCondyleFull.json"))
    LandmarkIO.writeLandmarksJson(relevantland.medialCondylePosterior,new File("relevant_feature_predicted/medialCondylePosterior.json"))
    LandmarkIO.writeLandmarksJson(relevantland.medialEpicondyle,new File("relevant_feature_predicted/medialEpicondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.medialSphere,new File("relevant_feature_predicted/medialSphere.json"))
    LandmarkIO.writeLandmarksJson(relevantland.whiteSideLine,new File("relevant_feature_predicted/whiteSideLine.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostAntPointOnLateralCondyle,new File("relevant_feature_predicted/mostAntPointOnLateralCondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostAntPointOnMedialCondyle,new File("relevant_feature_predicted/mostAntPointOnMedialCondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostDistPointOnLateralCondyle,new File("relevant_feature_predicted/mostDistPointOnLateralCondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostDistPointOnMedialCondyle,new File("relevant_feature_predicted/mostDistPointOnMedialCondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostPostPointOnLateralCondyle,new File("relevant_feature_predicted/mostPostPointOnLateralCondyle.json"))
    LandmarkIO.writeLandmarksJson(relevantland.mostPostPointOnMedialCondyle,new File("relevant_feature_predicted/mostPostPointOnMedialCondyle.json"))
  }

  case class relevantLand(medialEpicondyle:Seq[Landmark[_3D]],
                          lateralEpicondyle:Seq[Landmark[_3D]],
                          femoralHead:Seq[Landmark[_3D]],
                          distalShaft:Seq[Landmark[_3D]],
                         // apPoint:Seq[Landmark[_3D]],
                          anteriorInterCondyleGroove:Seq[Landmark[_3D]],
                          medialCondylePosterior:Seq[Landmark[_3D]] ,
                          lateralCondylePosterior:Seq[Landmark[_3D]],
                          medialCondyleAnterior:Seq[Landmark[_3D]],
                          lateralCondyleAnterior:Seq[Landmark[_3D]],
                          medialCondyleFull:Seq[Landmark[_3D]],
                          lateralCondyleFull:Seq[Landmark[_3D]],
                          medialSphere:Seq[Landmark[_3D]],
                          lateralSphere:Seq[Landmark[_3D]],
                          whiteSideLine:Seq[Landmark[_3D]],
                          mostAntPointOnLateralCondyle:Seq[Landmark[_3D]],
                          mostAntPointOnMedialCondyle:Seq[Landmark[_3D]],
                          mostDistPointOnLateralCondyle:Seq[Landmark[_3D]],
                          mostDistPointOnMedialCondyle:Seq[Landmark[_3D]],
                          mostPostPointOnLateralCondyle:Seq[Landmark[_3D]],
                          mostPostPointOnMedialCondyle:Seq[Landmark[_3D]]
                         )

}
