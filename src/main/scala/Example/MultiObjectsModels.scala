package Example

import java.io.File

import ShapeAndPoseModels._
import scalismo.common.{DiscreteField, PointId}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.Translation
import scalismo.ui.api.ScalismoUI

object MultiObjectsModels {

  def main(args: Array[String]): Unit = {
    scalismo.initialize()
    implicit val rng = scalismo.utils.Random(42)
    val ui = ScalismoUI()


  val dataGroup = ui.createGroup("datasets")

  val firstObjectMeshFiles = new java.io.File("E:\\PhD folders\\data for PGA model 21-03-2019\\lollipop data\\Online tutorial\\first_objects\\").listFiles
  val secondObjectMeshFiles = new java.io.File("E:\\PhD folders\\data for PGA model 21-03-2019\\lollipop data\\Online tutorial\\second_objects\\").listFiles

  val firstObjectRotationCenterFiles = new java.io.File("E:\\PhD folders\\data for PGA model 21-03-2019\\lollipop data\\Online tutorial\\rotation_centers_first_object\\").listFiles
  val secondObjectRotationCenterFiles = new java.io.File("E:\\PhD folders\\data for PGA model 21-03-2019\\lollipop data\\Online tutorial\\rotation_centers_second_object\\").listFiles

  val dataFiles = for (i<- 0 to firstObjectMeshFiles.size-1) yield {
    val firstObjectMesh = MeshIO.readMesh(firstObjectMeshFiles(i)).get
    val secondObjectMesh = MeshIO.readMesh(secondObjectMeshFiles(i)).get

    val firstObjectRotCent=LandmarkIO.readLandmarksJson[_3D](firstObjectRotationCenterFiles(i)).get
    val secondObjectRotCent=LandmarkIO.readLandmarksJson[_3D](secondObjectRotationCenterFiles(i)).get

    val firstObjectMeshView = ui.show(dataGroup, firstObjectMesh, "firstObjectMesh"+i)
    val secondObjectMeshView = ui.show(dataGroup, secondObjectMesh, "secondObjectMesh"+i)

    val firstObjectRotCentView = ui.show(dataGroup,firstObjectRotCent, "firstObjectRotCent")
    val secondObjectRotCentView = ui.show(dataGroup,secondObjectRotCent, "secondObjectRotCent")

    (List(firstObjectMesh, secondObjectMesh),
      List(firstObjectRotCent.head.point, secondObjectRotCent.head.point),
      List(firstObjectMeshView,secondObjectMeshView),
      List(firstObjectRotCentView,secondObjectRotCentView)
    ) // return a tuple of the mesh and roation centers with their associated view
  }


    val firstObjectReference=dataFiles.head._1(0)
    val secondObjectReference=dataFiles.head._1(1)
    val firstObjectRotCentRef=dataFiles.head._2(0)
    val secondObjectRotCentRef=dataFiles.head._2(1)

    val referenceMultibodyObject: MultiBodyObject[_3D, TriangleMesh] =  MultiBodyObject(
      List(firstObjectReference,secondObjectReference),
      List(firstObjectRotCentRef,secondObjectRotCentRef),
      List(Translation(EuclideanVector3D(0.0, 0.1, 0.0)).apply(firstObjectRotCentRef),Translation(EuclideanVector3D(0.0, 0.1, 0.0)).apply(secondObjectRotCentRef))

      )

    val expLog=MultiObjectPoseExpLogMapping(referenceMultibodyObject)

    val defFields = for (i <- 0 to dataFiles.size - 1) yield {

      val targMultiBody=
        MultiBodyObject(dataFiles(i)._1,
          dataFiles(i)._2,
          dataFiles(i)._2
        )

      val df = DiscreteField[_3D, MultiBodyObject[_3D, TriangleMesh]#DomainT, EuclideanVector[_3D]](referenceMultibodyObject, referenceMultibodyObject.pointSet.pointsWithId.toIndexedSeq.map(pt => targMultiBody.pointSet.point(pt._2) - pt._1))

      expLog.logMapping(df)
    }

    val continuousFields = defFields.map(f => f.interpolate(NearestNeighborInterpolator()))

    val multiBodyShapeAndPosePGA = MultiBodyShapeAndPosePGA(defFields,expLog)


    val sample=multiBodyShapeAndPosePGA.sample()
    val randomFirstObjectSample:TriangleMesh[_3D]=sample.objects(0)
    val randomSecondObjectSample:TriangleMesh[_3D]=sample.objects(1)
    val randomFirstObjectRotCent:Point[_3D]=sample.rotationCenters(0)
    val randomSecondObjectRotCent:Point[_3D]=sample.rotationCenters(1)

    ui.show(randomFirstObjectSample, "randomFirstObjectSample")
    ui.show(randomSecondObjectSample, "randomSecondObjectSample")
    ui.show(Seq(Landmark[_3D]("scapula",randomFirstObjectRotCent)), "randomFirstObjectRotCent")
    ui.show(Seq(Landmark[_3D]("humerus",randomSecondObjectRotCent)), "randomSecondObjectRotCent")


    val referenceDomainWithPoseParam=DomainWithPoseParameters(secondObjectReference,
      secondObjectRotCentRef,
      Translation(EuclideanVector3D(0.0, 0.1, 0.0)).apply(secondObjectRotCentRef)
    )


    val singleExpLog=SinglePoseExpLogMapping(referenceDomainWithPoseParam)
    val secondObjectTransitionModel:ShapeAndPosePGA[TriangleMesh]=multiBodyShapeAndPosePGA.transitionToSingleObject(referenceDomainWithPoseParam,singleExpLog)

    val singleObjectsample=secondObjectTransitionModel.sample()
    val secondObjectSample:TriangleMesh[_3D]=singleObjectsample.domain
    ui.show(secondObjectSample, "secondObjectSample transition model")



    //in this example we assumed that we observed the shape of the joint at 3rd position in training dataset

  }




}
