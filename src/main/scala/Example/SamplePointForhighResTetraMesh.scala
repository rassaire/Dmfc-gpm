package Example
import java.io.File

import ShapePoseAndIntesnityModels.DomainWithPoseAndIntensity.WarperDomainWithPoseAndIntensity
import ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity.WarperMultiBodyObjectWithIntensity
import ShapePoseAndIntesnityModels._
import ShapePoseAndIntesnityModels.io.MultibodyPGAIO
import breeze.linalg.DenseVector
import scalismo.common.DiscreteField
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Landmark, _3D}
import scalismo.io.{ImageIO, LandmarkIO, MeshIO}
import scalismo.mesh.{ScalarVolumeMeshField, TetrahedralMesh, TriangleMesh}
import scalismo.transformations.{Translation, Translation3D, TranslationAfterRotation3D}
import scalismo.common._
import scalismo.common.interpolation.{BarycentricInterpolator3D, NearestNeighborInterpolator, NearestNeighborInterpolator3D}
import scalismo.numerics.{UniformSampler, UniformTetrahedralMeshSampler3D, ValueInterpolator}
import scalismo.registration.LandmarkRegistration
import scalismo.ui.api.{Group, ScalismoUI}
import scalismo.utils.Random.implicits._
object SamplePointForhighResTetraMesh {
  def main(args: Array[String]): Unit = {
    val ui=ScalismoUI()

    val volume = ImageIO.read3DScalarImageAsType[Short](new File("E:\\PostDoc_Inserm\\Cars conference\\to_be _deleted\\knee.nii")).get
    print("read volume")
    ui.show(volume,"volume")
    //val referenceMesh3= MeshIO.readScalarVolumeMeshField[Double](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\scapula\\SW1-031M-RS.vtk")).get
//
//
//    //val surfMesh=referenceMesh3.domain.operations.getOuterSurface
//    //val refinedMesh=MeshIO.readTetrahedralMesh(new File("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\scapulaVirtualVolume-refined.inp")).get
//    //MeshIO.writeMesh(surfMesh,new File("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\dirty files\\refMeshSurf.stl"))
//    //ui.show(refinedMesh,"mesh")
//    val refTetraMeshhr=MeshIO.readTetrahedralMesh(new File ("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\dirty_files\\scapulaVirtualVolume.inp")).get
//    val inter = BarycentricInterpolator3D[Double]()
//    //val interpoltedRefmesh=referenceMesh3.interpolate(NearestNeighborInterpolator3D())
//    val interpoltedRefmesh=referenceMesh3.interpolate(inter)
//    val data=refTetraMeshhr.pointSet.points.toIndexedSeq.map(p=>interpoltedRefmesh.f(p))
//
//    //val data=refTetraMeshhr.pointSet.points.toIndexedSeq.map(p=>referenceMesh3.data(referenceMesh3.domain.pointSet.findClosestPoint(p).id.id))
//
//    val scalarMesh=scalismo.mesh.ScalarVolumeMeshField(refTetraMeshhr,data)
//
//    MeshIO.writeScalarVolumeMeshField(scalarMesh,new File("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\dirty_files\\refScalarVolumeMeshhr.vtk"))
//
//
//    println("number of points",refTetraMeshhr.pointSet.numberOfPoints)
//    println("number of tetras", refTetraMeshhr.tetrahedralization.tetrahedrons.size)
//
//    ui.show(scalarMesh,"mesh")
//  val rotcenter3= LandmarkIO.readLandmarksJson[_3D](new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\rotation centers\\scapula\\SW1-031M-RS.json")).get
//
//    val referenceMesh3=MeshIO.readScalarVolumeMeshField[Double](new File("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\dirty_files\\refScalarVolumeMeshhr.vtk")).get
//
//    val newDomain = ShapePoseAndIntesnityModels.MultiBodyObjectWithIntensity(
//      List(referenceMesh3.domain),
//      List(referenceMesh3.data),
//      List(rotcenter3.head.point),
//      List(Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(rotcenter3.head.point)
//      )
//    )
//    val readModel=MultibodyPGAIO.readShapeAndPoseMultiBodyWithIntensityPGA(new File("E:\\PhD folders\\data for PGA model 21-03-2019\\Full shoulder data\\test code data\\TestModel.h5")).get
//    val interpolatedGP =NearestNeighborInterpolator3D[MultiBodyObjectWithIntensity[_3D, TetrahedralMesh]#DomainT, ShapePoseAndIntensityVector[_3D]]() //readModel.gp.interpolate(NearestNeighborInterpolator3D())
//    val expLog = ShapePoseAndIntesnityModels.MultiObjectPosewithIntensityExpLogMapping(newDomain)
//    val highResModel=readModel.newReference(newDomain,interpolatedGP,expLog)
//
//    MultibodyPGAIO.writeShapeAndPoseMultiBodyWithIntensityPGA(highResModel,new File("E:\\PostDoc_Inserm\\vae_project\\dirt-test\\data\\testModelHr.h5"))
//
//    val mean=highResModel.mean.objects(0).pointSet.numberOfPoints
//    println("nber pts",mean)




  }
}
