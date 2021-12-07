package Example

import java.io.File

import Example.ShapePoseAndIntensityModels.visualizing
import ShapePoseAndIntesnityModels.DomainWithPoseAndIntensity.WarperDomainWithPoseAndIntensity
import ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity.WarperMultiBodyObjectWithIntensity
import ShapePoseAndIntesnityModels.MultiBodyShapePoseAndIntensityPGA
import ShapePoseAndIntesnityModels.io.MultibodyPGAIO
import breeze.linalg.DenseVector
import scalismo.geometry.{Landmark, Landmark3D, _3D}
import scalismo.mesh.{ScalarVolumeMeshField, TetrahedralMesh}
import scalismo.ui.api.ScalismoUI

object visualiseShapePoseAndIntensityModel {
  def main(args: Array[String]): Unit = {

    scalismo.initialize()
    val ui = ScalismoUI()
    implicit val rng = scalismo.utils.Random(42)
    implicit val warper1 = new WarperMultiBodyObjectWithIntensity[TetrahedralMesh]()
    implicit val warper2 = new WarperDomainWithPoseAndIntensity[TetrahedralMesh]()


    val readModel=MultibodyPGAIO.readShapeAndPoseMultiBodyWithIntensityPGA(new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\TestModel.h5")).get

    visualizing(readModel, 3,"readModel")
  }



  def visualizing(shapeAndPosePDM: MultiBodyShapePoseAndIntensityPGA[TetrahedralMesh], n:Int, infos:String)={

    val coeffs = DenseVector.ones[Double](shapeAndPosePDM.gp.rank)
    val aSample = shapeAndPosePDM.instance(coeffs)

    val ui = ScalismoUI()
    val g = ui.createGroup("mean and reference"+infos)
    //    ui.show(g, shapeAndPosePDM.gp.domain.objects(0), "ref object1")
    //    ui.show(g, shapeAndPosePDM.gp.domain.objects(1), "ref object2")

    ui.show(g, ScalarVolumeMeshField(aSample.objects(0),aSample.intensity(0)), " mean object1")
    //ui.show(g, Landmark3D("mean object1", aSample.rotationCenters(0)), " mean object1")
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
}