package Example

import java.io.File

import ShapePoseAndIntesnityModels.DomainWithPoseAndIntensity.WarperDomainWithPoseAndIntensity
import ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity.WarperMultiBodyObjectWithIntensity
import ShapePoseAndIntesnityModels._
import ShapePoseAndIntesnityModels.io.MultibodyPGAIO
import breeze.linalg.DenseVector
import scalismo.common.DiscreteField
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Landmark, _3D}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.{ScalarVolumeMeshField, TetrahedralMesh, TriangleMesh}
import scalismo.transformations.{Translation, Translation3D, TranslationAfterRotation3D}
import scalismo.common._
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.registration.LandmarkRegistration
import scalismo.ui.api.{Group, ScalismoUI}

object ShapePoseAndIntensityModels {


  def main(args: Array[String]): Unit = {

    scalismo.initialize()
    val ui=ScalismoUI()
    implicit val rng = scalismo.utils.Random(42)
    implicit val warper1=new WarperMultiBodyObjectWithIntensity[TetrahedralMesh]()
    implicit val warper2= new WarperDomainWithPoseAndIntensity[TetrahedralMesh]()

    //val st1: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\clavicle\\"
    //val st2: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\humerus\\"
    val st3: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\scapula\\"

    //val rot1: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\clavicle\\"
    //val rot2: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\humerus\\"
    val rot3: String = "E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\scapula\\"

    //val rotcenter1= LandmarkIO.readLandmarksJson[_3D](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\clavicle\\SW1-031M-RC.json")).get
    //val rotcenter2= LandmarkIO.readLandmarksJson[_3D](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\humerus\\SW1-031M-RH.json")).get
   val rotcenter3= LandmarkIO.readLandmarksJson[_3D](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\scapula\\SW1-031M-RS.json")).get

    //val referenceMesh1= MeshIO.readScalarVolumeMeshField[Double](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\clavicle\\SW1-031M-RC.vtk")).get
    //val referenceMesh2= MeshIO.readScalarVolumeMeshField[Double](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\humerus\\SW1-031M-RH.vtk")).get
    val referenceMesh3= MeshIO.readScalarVolumeMeshField[Double](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\scapula\\SW1-031M-RS.vtk")).get



//    val domain = ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity(
//      List(referenceMesh1.domain,referenceMesh2.domain,referenceMesh3.domain),
//      List(referenceMesh1.data,referenceMesh2.data,referenceMesh3.data),
//      List(rotcenter1.head.point,rotcenter2.head.point,rotcenter3.head.point),
//      List(Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(rotcenter1.head.point),
//        Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(rotcenter2.head.point),
//        Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(rotcenter3.head.point))
//    )
val domain = ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity(
  List(referenceMesh3.domain),
  List(referenceMesh3.data),
  List(rotcenter3.head.point),
  List(Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(rotcenter3.head.point)
  )
)

    val expLog = ShapePoseAndIntesnityModels.MultiObjectPosewithIntensityExpLogMapping(domain)




      val items = for (i <- 0 to new File(st3).listFiles().size - 1) yield {
        val g = ui.createGroup("example"+i)

        println("item " + i)
//        val mesh1 = MeshIO.readScalarVolumeMeshField[Double](new File(st1).listFiles().sortBy(_.getName).apply(i)).get
//        val rotCent1 = LandmarkIO.readLandmarksJson[_3D](new File(rot1).listFiles().sortBy(_.getName).apply(i)).get

        //val mesh2 = MeshIO.readScalarVolumeMeshField[Double](new File(st2).listFiles().sortBy(_.getName).apply(i)).get
        //val rotCent2 = LandmarkIO.readLandmarksJson[_3D](new File(rot2).listFiles().sortBy(_.getName).apply(i)).get

        val mesh3 = MeshIO.readScalarVolumeMeshField[Double](new File(st3).listFiles().sortBy(_.getName).apply(i)).get
        val rotCent3 = LandmarkIO.readLandmarksJson[_3D](new File(rot3).listFiles().sortBy(_.getName).apply(i)).get
//        ui.show(g,mesh1,"original mesh")
//        ui.show(g,mesh2,"original mesh")
//        ui.show(g,mesh3,"original mesh")

//        val targetDomain = ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity(
//          List(mesh1.domain,mesh2.domain,mesh3.domain),
//          List(mesh1.data,mesh2.data,mesh3.data),
//          List(rotCent1.head.point,rotCent2.head.point,rotCent3.head.point),
//          List(rotCent1.head.point,rotCent2.head.point,rotCent3.head.point)
//        )
val targetDomain = ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity(
  List(mesh3.domain),
  List(mesh3.data),
  List(rotCent3.head.point),
  List(rotCent3.head.point)
)


        val df = DiscreteField[_3D, ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, PointWithIntensityVectorVector[_3D]](domain, domain.pointSet.pointsWithId.toIndexedSeq.map(pt =>
          PointWithIntensityVectorVector(targetDomain.pointSet.point(pt._2) - pt._1, targetDomain.intensityDomain(pt._2.id) - domain.intensityDomain(pt._2.id))))

        expLog.logMapping(df)
        //log(domain,df,ui,g)
      }


    val model=MultiBodyShapePoseAndIntensityPGA(items,expLog)
    MultibodyPGAIO.writeShapeAndPoseMultiBodyWithIntensityPGA(model,new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\TestModel.h5"))
    val readModel=MultibodyPGAIO.readShapeAndPoseMultiBodyWithIntensityPGA(new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\TestModel.h5")).get
print("saved model")
    visualizing(model, 3,"")
    visualizing(readModel, 3,"readModel")
//    val sample=model.sample()
//    val scalarMeshSample=ScalarVolumeMeshField(sample.objects(0),sample.intensity(0))
//    ui.show(scalarMeshSample,"scalar mesh")
//
//    val sample1=model.sample()
//    val scalarMeshSample1=ScalarVolumeMeshField(sample1.objects(0),sample1.intensity(0))
//    ui.show(scalarMeshSample1,"scalar mesh1")
//
//    val shapePoseModel=model.PoseAndPoseModel
//    val shapePoseSample=shapePoseModel.sample()
//    ui.show(shapePoseSample.objects(0),"mesh")

  }

  def visualizing(shapeAndPosePDM: MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh], n:Int, infos:String)={

    val coeffs = DenseVector.ones[Double](shapeAndPosePDM.gp.rank)
    val aSample = shapeAndPosePDM.instance(coeffs)

    val ui = ScalismoUI()
    val g = ui.createGroup("mean and reference"+infos)
    //    ui.show(g, shapeAndPosePDM.gp.domain.objects(0), "ref object1")
    //    ui.show(g, shapeAndPosePDM.gp.domain.objects(1), "ref object2")
    ui.show(g, ScalarVolumeMeshField(aSample.objects(0),aSample.intensity(0)), " mean object1")
    ui.show(g, ScalarVolumeMeshField(aSample.objects(1),aSample.intensity(1)), " mean object2")
    ui.show(g, ScalarVolumeMeshField(aSample.objects(2),aSample.intensity(2)), " mean object3")

    for (i<-0 to n-1){
      val pc = ui.createGroup("PC "+infos+i)
      val v1= DenseVector.zeros[Double](shapeAndPosePDM.gp.rank)
      v1(i) = 2.0
      val Samplev1 = shapeAndPosePDM.instance(v1)

      //      val v2= DenseVector.zeros[Double](shapeAndPosePDM.gp.rank)
      //      v2(i) = -0.5
      //      val Samplev2 = shapeAndPosePDM.instance(v2)
      //
      //      val v3= DenseVector.zeros[Double](shapeAndPosePDM.gp.rank)
      //      v3(i) = -1.5
      //      val Samplev3 = shapeAndPosePDM.instance(v3)



      ui.show(pc, ScalarVolumeMeshField(Samplev1.objects(0),Samplev1.intensity(0)), "object1 +3 std" )
      ui.show(pc, ScalarVolumeMeshField(Samplev1.objects(1),Samplev1.intensity(1)), "object2 +3 std" )
      ui.show(pc, ScalarVolumeMeshField(Samplev1.objects(2),Samplev1.intensity(2)), "object3 +3 std" )

      val v4= DenseVector.zeros[Double](shapeAndPosePDM.gp.rank)
      v4(i) = -1.0
      val Samplev4 = shapeAndPosePDM.instance(v4)
      //
      //      ui.show(pc, Samplev2.objects(0), "object1 -0.5 std")
      //      ui.show(pc, Samplev2.objects(1), "object2 -0.5 std")
      //
      //      ui.show(pc, Samplev3.objects(0), "object1 -1.5 std")
      //      ui.show(pc, Samplev3.objects(1), "object2 -1.5 std")

      ui.show(pc, ScalarVolumeMeshField(Samplev4.objects(0),Samplev4.intensity(0)), "object1 -3 std" )
      ui.show(pc, ScalarVolumeMeshField(Samplev4.objects(1),Samplev4.intensity(1)), "object2 -3 std" )
      ui.show(pc, ScalarVolumeMeshField(Samplev4.objects(2),Samplev4.intensity(2)), "object3 -3 std" )
    }

  }



 def log(domain: MultiBodyObjectWithIntensity[_3D, TetrahedralMesh],
                           df: DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, PointWithIntensityVectorVector[_3D]]
                         , ui:ScalismoUI, g: Group)(implicit warper: DomainWarp[_3D, TetrahedralMesh]): DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, ShapePoseAndIntensityVector[_3D]] = {

 def restrictToSubdomain(
                                    domain: TetrahedralMesh[_3D],
                                    warpField: DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, PointWithIntensityVectorVector[_3D]]
                                  ): DiscreteField[_3D, TetrahedralMesh,PointWithIntensityVectorVector[_3D]] = {

     val field = warpField.interpolate(NearestNeighborInterpolator())
     field.discretize(domain, PointWithIntensityVectorVector(EuclideanVector.zeros[_3D],0.0)) // outside value is not accessed. If it is, it should be an error
   }


    //computation of the list of pose parameters needed for log deformation field of the MultiBodyObjectWithIntensity
    val transformations = for (i <- 0 to df.domain.objects.size - 1) yield {
      val reference = df.domain.objects(i)
      val restrictedDF = restrictToSubdomain(reference, df)
      val restrictedDF1=DiscreteField(reference, restrictedDF.values.map(_.Vec).toIndexedSeq)
      val targMesh = warper.transformWithField(domain.objects(i), restrictedDF1)
      if (i==0) {
        ui.show(g,targMesh,"training mesh clavicle")
      }else if (i==1){
        ui.show(g,targMesh,"training mesh humerus")
      }else{
        ui.show(g,targMesh,"training mesh scapula")
      }
      //compute landmarks and rotation center for the rigid transformation computation
      val referenceLandmark =
        reference.pointSet.pointIds.map(pi => Landmark[_3D](pi.id.toString, reference.pointSet.point(pi))).toSeq
      val targetLandmark =
        reference.pointSet.pointsWithId.map(pi => Landmark[_3D](pi._2.id.toString, pi._1 + restrictedDF1(pi._2))).toSeq
      val targetRotationCenter = df.domain.rotationCenters(i) + df.interpolate(NearestNeighborInterpolator())(
        df.domain.rotationCenters(i)
      ).Vec
      val bestTransform =
        LandmarkRegistration.rigid3DLandmarkRegistration(targetLandmark, referenceLandmark, targetRotationCenter)
      //compute the pose transformation (then pose deformation field), which obtained from the rigid transform
      val translation = Translation3D(df.domain.rotationCenters(i) - targetRotationCenter)
      val poseTransform = TranslationAfterRotation3D(translation, bestTransform.rotation)
      if (i==0) {
        ui.show(g,reference.transform(poseTransform.inverse), "pose mesh clavicle")
      }else if (i==1){
        ui.show(g,reference.transform(poseTransform.inverse), "pose mesh humerus")
      }else{
        ui.show(g,reference.transform(poseTransform.inverse), "pose mesh scapula")
      }
      val poseDF = DiscreteField(
        reference,
        reference.pointSet.points.map(pt => poseTransform.inverse.apply(pt) - pt).toIndexedSeq
      )

      if (i==0) {
        ui.show(g,targMesh.transform(poseTransform),"aligned mesh clavicle")
      }else if (i==1){
        ui.show(g,targMesh.transform(poseTransform),"aligned mesh humerus")
      }else{
        ui.show(g,targMesh.transform(poseTransform),"aligned mesh scapula")
      }

      //compute shape deformation field
      val shapeDF = DiscreteField(reference,
        reference.pointSet.pointsWithId
          .map(pt => PointWithIntensityVectorVector[_3D](bestTransform.apply(targMesh.pointSet.point(pt._2)) - pt._1,restrictedDF(pt._2).Intensity))
          .toIndexedSeq)
      //compute the neutral point required for the retrieval of the neutral position of the shape
      val targetNeutralPoint = bestTransform(
        df.domain.neutralPoints(i) + df.interpolate(NearestNeighborInterpolator())(df.domain.neutralPoints(i)).Vec
      )
      (shapeDF, poseDF, targetRotationCenter, targetNeutralPoint)
    }
    //computation of the log vector fields, with associated vector having shape and pose components
    val data = (for (i <- 0 to transformations.size - 1) yield {
      transformations(i)._1.pointsWithIds
        .map(pi => ShapePoseAndIntensityVector(transformations(i)._1.apply(pi._2), transformations(i)._2.apply(pi._2)))
        .toIndexedSeq ++
        IndexedSeq(
          ShapePoseAndIntensityVector(PointWithIntensityVectorVector[_3D](transformations(i)._3 - df.domain.rotationCenters(i),0.0),
            transformations(i)._3 - df.domain.rotationCenters(i))
        ) ++
        IndexedSeq(
          ShapePoseAndIntensityVector(PointWithIntensityVectorVector[_3D](transformations(i)._4 - df.domain.neutralPoints(i),0.0),
            transformations(i)._4 - df.domain.neutralPoints(i))
        )
    }).reduce((acc, s) => acc ++ s)
    DiscreteField[_3D, MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, ShapePoseAndIntensityVector[_3D]](df.domain, data)

  }
}