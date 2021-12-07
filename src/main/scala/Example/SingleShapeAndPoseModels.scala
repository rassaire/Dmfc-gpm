package Example
import ShapeAndPoseModels.{DomainWithPoseParameters, ShapeAndPosePGA, SinglePoseExpLogMapping}
import scalismo.common.{DiscreteField, PointId}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.Translation
import scalismo.ui.api.ScalismoUI

object SingleShapeAndPoseModels {
  def main(args: Array[String]): Unit = {
    scalismo.initialize()
    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)
    val ui = ScalismoUI()

    // Loading and preprocessing a dataset:

    val dataGroup = ui.createGroup("datasets")

    val meshFiles = new java.io.File("path/humeri/").listFiles
    val rotationCenterFiles = new java.io.File("path/humeri/").listFiles

    val dataFiles = for (i<- 0 to meshFiles.size-1) yield {
      val mesh = MeshIO.readMesh(meshFiles(i)).get
      val rotcent=LandmarkIO.readLandmarksJson[_3D](rotationCenterFiles(i)).get
      val meshView = ui.show(dataGroup, mesh, "mesh")
      val rotCentView = ui.show(dataGroup, mesh, "mesh")
      (mesh, rotcent, meshView, rotCentView) // return a tuple of the mesh and roation centers with their associated view
    }

    //Building logarithmic functions from data

    val reference = dataFiles.head._1
    val refRotCent=dataFiles.head._2


    val referenceDomainWithPoseParam=DomainWithPoseParameters(reference,
      refRotCent.head.point,
      Translation(EuclideanVector3D(0.0, 0.1, 0.0)).apply(refRotCent.head.point)
    )

    val singleExpLog=SinglePoseExpLogMapping(referenceDomainWithPoseParam)

    val defFields = for (i <- 0 to dataFiles.size - 1) yield {

      val targDomainWithPoseParam=DomainWithPoseParameters(dataFiles(i)._1,
        dataFiles(i)._2.head.point,
        dataFiles(i)._2.head.point
      )

      val df=DiscreteField[_3D, ({ type T[D] = DomainWithPoseParameters[D, TriangleMesh] })#T, EuclideanVector[_3D]](referenceDomainWithPoseParam,
        referenceDomainWithPoseParam.pointSet.pointsWithId.toIndexedSeq.map(pt =>targDomainWithPoseParam.pointSet.point(pt._2) - pt._1))

      singleExpLog.logMappingSingleDomain(df)

    }
    //Note that the deformation fields can be be interpolated through nearest-neighbourhood interpolation
    // to make sure that they are defined on all the points of the reference mesh.

    val continuousFields = defFields.map(f => f.interpolate(NearestNeighborInterpolator()))

    //Single shape and pose model building
    val shapeAndPoseModel: ShapeAndPosePGA[TriangleMesh] = ShapeAndPosePGA(defFields,singleExpLog)


    //Single shape and pose model sampling
    val sample=shapeAndPoseModel.sample()
    val randomMeshSample:TriangleMesh[_3D]=sample.domain
    val randomRotCent:Point[_3D]=sample.rotCenter
    ui.show(randomMeshSample, "randomMeshSample")
    ui.show(Seq(Landmark[_3D]("",randomRotCent)), "randomRotCent")

    //Single shape and pose model marginalisation
    val MarginalshapeModel: PointDistributionModel[_3D, TriangleMesh]=shapeAndPoseModel.shapePDM
    val MarginalPoseFromShapeAndPose: ShapeAndPosePGA[TriangleMesh]=shapeAndPoseModel.PosePGA

    //Single shape ad pose model posterior

    val littleNoise=0.2 // variance of the Gaussian noise N(0,0.2)
    val observedData:IndexedSeq[(PointId, Point[_3D])]= ???
    val ShapeAndPosePosterior: ShapeAndPosePGA[TriangleMesh] = shapeAndPoseModel.posterior(observedData,littleNoise)
  }



}
