//package FittingWithMCMC
//
//import java.awt.Color
//import java.io.File
//
//import scalismo.common.ScalarMeshField
//
//import ShapeAndPoseModels._
////import Application.fitting.runMCMC
//import breeze.linalg.{DenseMatrix, DenseVector}
//import scalismo.common.interpolation.NearestNeighborInterpolator
//import scalismo.geometry._
//import scalismo.io.{LandmarkIO, MeshIO}
//import scalismo.mesh.{MeshMetrics, TriangleMesh}
//import scalismo.sampling.TransitionProbability
//import scalismo.sampling.algorithms.MetropolisHastings
//import scalismo.sampling.evaluators.ProductEvaluator
//import scalismo.sampling.proposals.MixtureProposal
//import scalismo.statisticalmodel.MultivariateNormalDistribution
//import scalismo.ui.api.ScalismoUI
//
//object FittingWithMultiShapePoseModel {
//
//  def main (args: Array[String]): Unit= {
//
//    scalismo.initialize()
//    implicit val rng = scalismo.utils.Random(42)
//    val ui = ScalismoUI()
//    val t1 = System.nanoTime
//    val list= readRelevantLandList()
//    val refRelevandLand= new relevantLand(
//    list(0),list(1),list(2),list(3),list(4),list(5),
//    list(6),list(7),list(8),list(9),list(10),list(11),
//    list(12),list(13),list(14),list(15), list(16),list(17),
//    list(18),list(19))
//
//
////    val targetMesh1: TriangleMesh[_3D] = MeshIO.readMesh(new File("data\\femur.stl")).get
////    val targetMesh2: TriangleMesh[_3D] = MeshIO.readMesh(new File("data\\tibia.stl")).get
//
//    val ShapeAndPosePGA = io.MultibodyPGAIO.readShapeAndPoseMultiBodyPGA(new File("E:\\PostDoc_Inserm\\knee data\\data in correspondence\\knee model sampling\\shapePoseModelPGAKnee.h5")).get
////    val refFemur = ShapeAndPosePGA.gp.domain.objects.head.pointSet.pointsWithId.toSeq.map(pi=>Landmark[_3D](pi._2.id.toString,pi._1))
////    val meanfemur= ShapeAndPosePGA.mean.objects.head.pointSet.pointsWithId.toSeq.map(pi=>Landmark[_3D](pi._2.id.toString,pi._1))
////
////    val t=LandmarkRegistration.rigid3DLandmarkRegistration(refFemur,meanfemur,refFemur.head.point)
//    val num="021"
//    val targetMesh1: TriangleMesh[_3D] = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\partial_knees\\"+num+"_2_femur\\femurAlignedToModel.stl")).get//.transform(t)
//
//    val targetMesh2: TriangleMesh[_3D] = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\partial_knees\\"+num+"_2_tibia\\tibiaAlignedToModel.stl")).get//.transform(t)
//
//    val groundTruth = MeshIO.readMesh(new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\ground_truth\\"+num+"_2_KneeAlignedToModel.stl")).get
//    //val ShapeAndPosePGA = MultibodyPDMIO.readShapeAndPoseMultiBodyPDM(new File("E:\\PostDoc_Inserm\\knee data\\data in correspondence\\knee model sampling\\shapePoseModelPGAKnee.h5")).get
//    //TestModels.visualizingSSM(ShapeAndPosePGA.shapePDM,10)
//
//    val targetGroup = ui.createGroup("target meshes")
//    val targetView1 = ui.show(targetGroup, targetMesh1, "target mesh")
//    val targetView2 = ui.show(targetGroup, targetMesh2, "target mesh")
//    targetView1.color = Color.blue
//    targetView2.color = Color.blue
//
//
//    val landmarkNoiseVariance = 1.0
//    val uncertainty = MultivariateNormalDistribution(
//      DenseVector.zeros[Double](3),
//      DenseMatrix.eye[Double](3) * landmarkNoiseVariance
//    )
////    val sample1=UniformMeshSampler3D(targetMesh1, targetMesh1.pointSet.numberOfPoints)
////    val sample2=UniformMeshSampler3D(targetMesh2, targetMesh1.pointSet.numberOfPoints)
//
//    //val targetPoints: Seq[(Point[_3D],MultivariateNormalDistribution)] = targetMesh1.pointSet.pointsWithId.map { pi => (pi._1,uncertainty )}.toSeq
//
//    val targetPoints: List[Seq[(Point[_3D], MultivariateNormalDistribution)]] = List(targetMesh1,targetMesh2).map(targetMesh=>targetMesh.pointSet.pointsWithId.map { pi => (pi._1,uncertainty) }.toSeq)
//    val model=ShapeAndPosePGA.gp.interpolate(NearestNeighborInterpolator()).truncate(20)
//
//    //val likelihoodEvaluator = CachedEvaluator(MCMC.ProximityEvaluatorMultiBodySingleObservation(ShapeAndPosePGA, targetPoints))
//    //implicit val rand = scalismo.utils.Random(42)
////    val distrDim1 = Uniform(0, targetPoints.size)(rng.breezeRandBasis)
////    val samplePoints =(0 until 2000).map(i => targetPoints(distrDim1.draw().toInt)).toList
//    val likelihoodEvaluator = CachedEvaluator(ProximityEvaluatorMultiBody(ShapeAndPosePGA, targetPoints))
////    val likelihoodEvaluator = CachedEvaluator(MCMC.ProximityEvaluatorContinuousMultiBody(model,
////      ShapeAndPosePGA.logExpMapping,
////      ShapeAndPosePGA.gp.domain,
////      targetPoints))
//
//
//    val shapeUpdateProposalLargeStep = ShapeUpdateProposal(ShapeAndPosePGA.gp.rank, 0.3)
//   // val shapeUpdateProposalSmallStep=ShapeUpdateProposal(ShapeAndPosePGA.gp.rank, 0.1)
//
//    val generator = MixtureProposal.fromProposalsWithTransition(
//      (1.0, shapeUpdateProposalLargeStep))//,
//     // (0.3, shapeUpdateProposalSmallStep))
//
//
//    val priorEvaluator = CachedEvaluator(PriorEvaluatorMultiBody(ShapeAndPosePGA))
//
//    val posteriorEvaluator = ProductEvaluator(priorEvaluator, likelihoodEvaluator)
//
//    runMCMC(ShapeAndPosePGA, generator, posteriorEvaluator, ui, groundTruth,refRelevandLand,num)
//    val duration = (System.nanoTime - t1) *1.66667e-11
//    println("running time "+duration)
//  }
//
//
//
//
//  def runMCMC(model:MultibodyShapeAndPosePDM[TriangleMesh],
//              generator:MixtureProposal[Sample] with TransitionProbability[Sample],
//              evaluators:ProductEvaluator[Sample],
//              ui:ScalismoUI,
//              targets:TriangleMesh[_3D],
//              refRelevantLand:relevantLand,
//              num:String
//             )={
//    implicit val rng = scalismo.utils.Random(42)
////    val initialParameters = Parameters(
////      DenseVector.zeros[Double](model.gp.rank)
////    )
//
//    val initialParameters = Parameters(
//      DenseVector.zeros[Double](model.gp.rank),
//      EuclideanVector(0, 0, 0),
//      (0.0, 0.0, 0.0),
//      EuclideanVector(0, 0, 0),
//      (0.0, 0.0, 0.0)
//    )
//
//    //val initialSample = Sample("initial", initialParameters)
//
//    val initialSample = Sample("initial", initialParameters,model.mean.rotationCenters(0),model.mean.rotationCenters(1))
//
//    val chain = MetropolisHastings(generator, evaluators)
//    val logger = new MixtureLogger()
//
//    val mhIterator = chain.iterator(initialSample, logger)
//
//    // val seq10k = mhIterator.take(600).toIndexedSeq
//
//
//
//    val samplingIterator = for ((sample, iteration) <- mhIterator.zipWithIndex) yield {
//      if (iteration % 100 == 0) {
//        ui.show(KneeModel.mergeMeshes(model.instance(sample.parameters.modelCoefficients).objects), "sample object")
//        //        ui.show(model.instance(sample.parameters.modelCoefficients).objects(1), "sample object 2")
//        //        ui.show(model.instance(sample.parameters.modelCoefficients).objects(2), "sample object 3")
//      }
//      println("iteration " + iteration)
//      sample
//    }
////    val mesh=DomainWithPoseParameters(model.gp.domain.objects(0),
////      model.gp.domain.rotationCenters(0),
////      model.gp.domain.neutralPoints(0)
////    )
////    val newmodel=model.transitionToSingleObject(mesh,SinglePoseExpLogMapping(mesh)).shapePDM
////    val newinitialSample = Sample("initial", initialParameters)
////
////
////    val newchain=chain.iterator(newinitialSample, logger)
//
//    val samples = samplingIterator.slice(800, 3500).toIndexedSeq
//    println(logger.acceptanceRatios())
//
//
//
//    val bestSample = samples.maxBy(evaluators.logValue)
//    val bestmesh=model.instance(bestSample.parameters.modelCoefficients).objects
//
//    //save relevantLands as csv files , for different regions of the femur
//    //saveRelevantLandList(refRelevantLand,model.gp.domain.objects(0),bestmesh(0))
//
//    val combinedMesh = KneeModel.mergeMeshes(bestmesh)
//
//    ui.show(combinedMesh,"best fit")
//    MeshIO.writeMesh(combinedMesh,new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\"+num+"_2_reconstructedKnee.stl"))
//    MeshIO.writeMesh(bestmesh.head,new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\"+num+"_2_reconstructedFemur.stl"))
//    MeshIO.writeMesh(bestmesh(1),new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\"+num+"_2_reconstructedTibia.stl"))
//
//
//
//    val dist=MeshMetrics.avgDistance(targets, combinedMesh)
//
//    val errorMesh = ScalarMeshField[Double](combinedMesh, combinedMesh.pointSet.points.toIndexedSeq.map{p=>
//      val closestP=targets.pointSet.findClosestPoint(p).point
//      (closestP-p).norm})
//
//    MeshIO.writeScalarMeshField[Double](errorMesh, new File("E:\\PostDoc_Inserm\\caos_2022\\experiments\\knee_reconstruction\\outputs\\error_meshes\\"+num+"_2_errorMesh.vtk"))
//
//    print(dist)
//    //MeshIO.writeMesh(combinedMesh,new File("E:\\PostDoc_Inserm\\Cropped knee\\reconstructedCombinedMeshAligned.stl"))
////
////    if (targets.size>1) {
////      Array((for (i <- 0 to bestmesh.size - 1) yield {
////        val dist = hausdorffDistance(targets(i),bestmesh(i))
////        println("HD of the object"+i+" is " + dist)
////        dist
////      }).max.toString)
////    }else{
////      val dist=hausdorffDistance(targets.head,bestmesh.head)
////      println("HD is " + dist)
////       Array(dist.toString)
////      }
//
//
//
//  }
//
//  /**
//    * Returns the Hausdorff distance between the two meshes
//    */
//  def hausdorffDistance(m1: TriangleMesh[_3D], m2: TriangleMesh[_3D]): Double = {
//    def allDistsBetweenMeshes(mm1: TriangleMesh[_3D], mm2: TriangleMesh[_3D]): Iterator[Double] = {
//      for (ptM1 <- mm1.pointSet.points) yield {
//        val cpM2 = mm2.operations.closestPointOnSurface(ptM1).point
//        (ptM1 - cpM2).norm
//      }
//    }
//
//    val d1 = allDistsBetweenMeshes(m1, m2)
//
//   d1.max
//
//  }
//
//  def readRelevantLandList():List[Seq[Landmark[_3D]]]={
//    val medialEpicondyle:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialEpicondyle.json")).get
//    val lateralEpicondyle:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralEpicondyle.json")).get
//    val femoralHead:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\femoralHead.json")).get
//    val distalShaft:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\distalShaft.json")).get
//   // val apPoint:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\apPoint.json")).get
//    val anteriorIntercondyleGroove:Seq[Landmark[_3D]] =LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\anteriorIntercondyleGroove.json")).get
//    val medialCondylePosterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondylePosterior.json")).get
//    val lateralCondylePosterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondylePosterior.json")).get
//    val medialCondyleAnterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondyleAnterior.json")).get
//    val lateralCondyleAnterior:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondyleAnterior.json")).get
//    val medialCondyleFull:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialCondyleFull.json")).get
//    val lateralCondyleFull:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralCondyleFull.json")).get
//    val medialSphere:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\medialSphere.json")).get
//    val lateralSphere:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\lateralSphere.json")).get
//    val whiteSideLine:Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\whiteSideLine.json")).get
//    val mostAntPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostAntPointOnLateralCondyle.json")).get
//    val mostAntPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostAntPointOnMedialCondyle.json")).get
//    val mostDistPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostDistPointOnLateralCondyle.json")).get
//    val mostDistPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostDistPointOnMedialCondyle.json")).get
//    val mostPostPointOnLateralCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostPostPointOnLateralCondyle.json")).get
//    val mostPostPointOnMedialCondyle:Seq[Landmark[_3D]]= LandmarkIO.readLandmarksJson3D(new File("E:\\PostDoc_Inserm\\relevant-anatomical-parameters\\reference _relevant _landmarks\\mostPostPointOnMedialCondyle.json")).get
//
//    List(medialEpicondyle,
//      lateralEpicondyle,
//      femoralHead,
//      distalShaft,
//     // apPoint,
//      anteriorIntercondyleGroove,
//      medialCondylePosterior,
//      lateralCondylePosterior,
//      medialCondyleAnterior,
//      lateralCondyleAnterior,
//      medialCondyleFull,
//      lateralCondyleFull,
//      medialSphere,
//      lateralSphere,
//      whiteSideLine,
//      mostAntPointOnLateralCondyle,
//      mostAntPointOnMedialCondyle,
//      mostDistPointOnLateralCondyle,
//      mostDistPointOnMedialCondyle,
//      mostPostPointOnLateralCondyle,
//      mostPostPointOnMedialCondyle
//    )
//  }
//
//  def saveRelevantLandList(relevantLand1:relevantLand, refmesh:TriangleMesh[_3D] ,bestFit:TriangleMesh[_3D]): Unit ={
//
//
//
//    val relevantland=relevantLand(
//
//      relevantLand1.medialEpicondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.lateralEpicondyle.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.femoralHead.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.distalShaft.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
////      relevantLand1.apPoint.map{
////        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
////          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.anteriorInterCondyleGroove.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.medialCondylePosterior.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.lateralCondylePosterior.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.medialCondyleAnterior.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.lateralCondyleAnterior.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.medialCondyleFull.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.lateralCondyleFull.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.medialSphere.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.lateralSphere.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.whiteSideLine.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//          Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.mostAntPointOnLateralCondyle.map{
//        l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//    relevantLand1.mostAntPointOnMedialCondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.mostDistPointOnLateralCondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.mostDistPointOnMedialCondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.mostPostPointOnLateralCondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))},
//
//      relevantLand1.mostPostPointOnMedialCondyle.map{
//      l=>val id=refmesh.pointSet.findClosestPoint(l.point).id
//        Landmark[_3D](id.id.toString,bestFit.pointSet.point(id))}
//    )
//
//
//
//    LandmarkIO.writeLandmarksJson(relevantland.anteriorInterCondyleGroove,new File("relevant_feature_predicted/anteriorIntercondyleGroove.json"))
//   // LandmarkIO.writeLandmarksJson(relevantland.apPoint,new File("relevant_feature_predicted/apPoint.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.distalShaft,new File("relevant_feature_predicted/distalShaft.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.femoralHead,new File("relevant_feature_predicted/femoralHead.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.lateralCondyleAnterior,new File("relevant_feature_predicted/lateralCondyleAnterior.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.lateralCondyleFull,new File("relevant_feature_predicted/lateralCondyleFull.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.lateralCondylePosterior,new File("relevant_feature_predicted/lateralCondylePosterior.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.lateralEpicondyle,new File("relevant_feature_predicted/lateralEpicondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.lateralSphere,new File("relevant_feature_predicted/lateralSphere.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.medialCondyleAnterior,new File("relevant_feature_predicted/medialCondyleAnterior.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.medialCondyleFull,new File("relevant_feature_predicted/medialCondyleFull.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.medialCondylePosterior,new File("relevant_feature_predicted/medialCondylePosterior.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.medialEpicondyle,new File("relevant_feature_predicted/medialEpicondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.medialSphere,new File("relevant_feature_predicted/medialSphere.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.whiteSideLine,new File("relevant_feature_predicted/whiteSideLine.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostAntPointOnLateralCondyle,new File("relevant_feature_predicted/mostAntPointOnLateralCondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostAntPointOnMedialCondyle,new File("relevant_feature_predicted/mostAntPointOnMedialCondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostDistPointOnLateralCondyle,new File("relevant_feature_predicted/mostDistPointOnLateralCondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostDistPointOnMedialCondyle,new File("relevant_feature_predicted/mostDistPointOnMedialCondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostPostPointOnLateralCondyle,new File("relevant_feature_predicted/mostPostPointOnLateralCondyle.json"))
//    LandmarkIO.writeLandmarksJson(relevantland.mostPostPointOnMedialCondyle,new File("relevant_feature_predicted/mostPostPointOnMedialCondyle.json"))
//  }
//
//  case class relevantLand(medialEpicondyle:Seq[Landmark[_3D]],
//                          lateralEpicondyle:Seq[Landmark[_3D]],
//                          femoralHead:Seq[Landmark[_3D]],
//                          distalShaft:Seq[Landmark[_3D]],
//                         // apPoint:Seq[Landmark[_3D]],
//                          anteriorInterCondyleGroove:Seq[Landmark[_3D]],
//                          medialCondylePosterior:Seq[Landmark[_3D]] ,
//                          lateralCondylePosterior:Seq[Landmark[_3D]],
//                          medialCondyleAnterior:Seq[Landmark[_3D]],
//                          lateralCondyleAnterior:Seq[Landmark[_3D]],
//                          medialCondyleFull:Seq[Landmark[_3D]],
//                          lateralCondyleFull:Seq[Landmark[_3D]],
//                          medialSphere:Seq[Landmark[_3D]],
//                          lateralSphere:Seq[Landmark[_3D]],
//                          whiteSideLine:Seq[Landmark[_3D]],
//                          mostAntPointOnLateralCondyle:Seq[Landmark[_3D]],
//                          mostAntPointOnMedialCondyle:Seq[Landmark[_3D]],
//                          mostDistPointOnLateralCondyle:Seq[Landmark[_3D]],
//                          mostDistPointOnMedialCondyle:Seq[Landmark[_3D]],
//                          mostPostPointOnLateralCondyle:Seq[Landmark[_3D]],
//                          mostPostPointOnMedialCondyle:Seq[Landmark[_3D]]
//                         )
//
//}
