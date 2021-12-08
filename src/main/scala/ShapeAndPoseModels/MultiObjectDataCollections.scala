package ShapeAndPoseModels
import java.io.File
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteDomain, DiscreteField, PointId}
import scalismo.geometry.{_3D, EuclideanVector, EuclideanVector3D, Landmark}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.transformations.Translation


class DataCollectionDomainWithPoseParameters[D, DDomain[D] <: DomainWithPoseParameters[D, DDomain], Value](
  dataItems: Seq[DiscreteField[D, DDomain, Value]]
) {
  require(dataItems.size > 0 && dataItems.forall(di => di.domain == dataItems.head.domain))

  val size: Int = dataItems.size

  def reference: DDomain[D] = dataItems.head.domain

}

object DataCollectionDomainWithPoseParameters {
  type TriangleMeshDataCollectionDomainWithPoseParameters[D] =
    DataCollection[D, DomainWithPoseParameters[D, TriangleMesh]#DomainT, ShapeAndPoseVector[D]]

  private def differenceFieldToReference[D, DDomain[D] <: DiscreteDomain[D]](
    reference: DDomain[D],
    mesh: DDomain[D]
  ): DiscreteField[D, DDomain, EuclideanVector[D]] = {
    require(reference.pointSet.numberOfPoints == mesh.pointSet.numberOfPoints)

    DiscreteField(reference, reference.pointSet.pointsWithId.toIndexedSeq.map(pt => mesh.pointSet.point(pt._2) - pt._1))

  }
  def fromTriangleMesh3DSequence(
    referenceMesh: TriangleMesh[_3D],
    referenceRotCenter: Seq[Landmark[_3D]],
    meshes: Seq[TriangleMesh[_3D]],
    rotCenters: Seq[Seq[Landmark[_3D]]]
  ): TriangleMeshDataCollectionDomainWithPoseParameters[_3D] = {

    val SingleDomainWithPoseParameters = DomainWithPoseParameters(
      referenceMesh,
      referenceRotCenter.head.point,
      Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(referenceRotCenter.head.point)
    )
    val singleExpLog = SinglePoseExpLogMapping(SingleDomainWithPoseParameters)
    val dfs = for (i <- 0 to meshes.size - 1) yield {

      val targDomainWithPoseParameters =
        DomainWithPoseParameters(meshes(i), rotCenters(i).head.point, rotCenters(i).head.point)
      val df = differenceFieldToReference[_3D, DomainWithPoseParameters[_3D, TriangleMesh]#DomainT](
        SingleDomainWithPoseParameters,
        targDomainWithPoseParameters
      )
      singleExpLog.logMappingSingleDomain(df)
    }
    new DataCollection(dfs)
  }

  def fromDirectories(referenceMesh: TriangleMesh[_3D],
                      referenceRotCenter: Seq[Landmark[_3D]],
                      files: File,
                      rotCenters: File): TriangleMeshDataCollectionDomainWithPoseParameters[_3D] = {

    val n = files.listFiles().size
    def testDataSize(files: File, rotCenters: File): Boolean = {
      var bool: Boolean = true
      if (rotCenters.listFiles().size == n) {
        bool = true
      } else {
        bool = false
      }
      bool
    }
    if (testDataSize(files, rotCenters)) {
      val data = for (i <- 0 to n - 1) yield {
        val mesh = MeshIO.readMesh(files.listFiles().sortBy(_.getName).apply(i)).get
        val rotCent = LandmarkIO.readLandmarksJson[_3D](rotCenters.listFiles().sortBy(_.getName).apply(i)).get
        (mesh, rotCent)
      }
      fromTriangleMesh3DSequence(referenceMesh, referenceRotCenter, data.map(d => d._1), data.map(d => d._2))
    } else {
      throw new Exception("number of objects should be the same as number of rotcenter")
    }
  }
  /*
   * Performs a Generalized Procrustes Analysis on the data collection.
   * This is done by repeatedly computing the mean of all DomainWithPoseParameters[TriangleMesh] in the dataset and
   * aligning all items rigidly to the mean.
   *
   * The reference DomainWithPoseParameters[TriangleMesh]  is unchanged, only the transformations in the collection are adapted
   */
  def gpa(dc: TriangleMeshDataCollectionDomainWithPoseParameters[_3D])(
    ): TriangleMeshDataCollectionDomainWithPoseParameters[_3D] = {
    computeGPA(dc)
  }

  private def computeGPA(
    dc1: TriangleMeshDataCollectionDomainWithPoseParameters[_3D]
  ): TriangleMeshDataCollectionDomainWithPoseParameters[_3D] = {
    implicit val rng = scalismo.utils.Random(42)
    val items = dc1.dataItems
    val meshes = for (df <- items) yield {

      val warpFieldInterpolated = df.interpolate(NearestNeighborInterpolator())
      val restrictedDF = DiscreteField(df.domain.domain, warpFieldInterpolated.andThen(_.shapeVec))
      TriangleMesh.domainWarp3D.transformWithField(df.domain.domain, restrictedDF)
    }
    val dc = DataCollection.fromTriangleMesh3DSequence(items(0).domain.domain, meshes)
    val gpaDc = DataCollection.gpa(dc)

    val data = for (i <- 0 to dc.size - 1) yield {
      val fieldInterpolated = items(i).interpolate(NearestNeighborInterpolator())
      val poseDF = DiscreteField(
        items(i).domain.domain,
        items(i).domain.domain.pointSet.points.toIndexedSeq.map(p => fieldInterpolated(p).poseVec)
      )

      DiscreteField[_3D, DomainWithPoseParameters[_3D, TriangleMesh]#DomainT, ShapeAndPoseVector[_3D]](
        items(0).domain,
        gpaDc
          .dataItems(i)
          .pointsWithIds
          .map(pi => ShapeAndPoseVector(gpaDc.dataItems(i).apply(pi._2), poseDF.apply(pi._2)))
          .toIndexedSeq ++
          IndexedSeq(fieldInterpolated(items(i).domain.rotCenter)) ++
          IndexedSeq(fieldInterpolated(items(i).domain.neutralPoint))
      )
    }

    new DataCollection(data)
  }

}

object DataCollectionMultiObjects {
  type TriangleMeshDataCollectionMultiObjects[D] =
    DataCollection[D, MultiBodyObject[D, TriangleMesh]#DomainT, ShapeAndPoseVector[D]]

  private def differenceFieldToReference[D, DDomain[D] <: DiscreteDomain[D]](
                                                                              reference: MultiBodyObject[D, DDomain],
                                                                              mesh: MultiBodyObject[D, DDomain]
  ): DiscreteField[D, MultiBodyObject[D, DDomain]#DomainT, EuclideanVector[D]] = {
    for (i <- 0 to reference.objects.size - 1) {
      require(reference.objects(i).pointSet.numberOfPoints == mesh.objects(i).pointSet.numberOfPoints)
    }

    DiscreteField[D, MultiBodyObject[D, DDomain]#DomainT, EuclideanVector[D]](
      reference,
      reference.pointSet.pointsWithId.toIndexedSeq.map(pt => mesh.pointSet.point(pt._2) - pt._1)
    )

  }
  def fromTriangleMesh3DSequence(
    referenceMesh: List[TriangleMesh[_3D]],
    referenceRotCenter: List[Seq[Landmark[_3D]]],
    meshes: List[Seq[TriangleMesh[_3D]]],
    rotCenters: List[Seq[Seq[Landmark[_3D]]]]
  ): TriangleMeshDataCollectionMultiObjects[_3D] = {

    val MultiObjectsDomain = MultiBodyObject(
      referenceMesh,
      referenceRotCenter.map(l => l.head.point),
      referenceRotCenter.map(l => Translation(EuclideanVector3D(0.0, 0.3, 0.0)).apply(l.head.point))
    )
    val ExpLog = MultiObjectPoseExpLogMapping(MultiObjectsDomain)
    val dfs = for (i <- 0 to meshes.size - 1) yield {

      val data = for (j <- 0 to meshes(0).size - 1) yield {
        (meshes(i)(j), rotCenters(i)(j).head.point, rotCenters(i)(j).head.point)
      }

      val targMultiObjectsDomain =
        MultiBodyObject(data.map(d => d._1).toList, data.map(d => d._2).toList, data.map(d => d._3).toList)

      val df = differenceFieldToReference(
        MultiObjectsDomain,
        targMultiObjectsDomain
      )
      ExpLog.logMapping(df)
    }
    new DataCollection(dfs)
  }

  def fromDirectories(referenceMesh: List[TriangleMesh[_3D]],
                      referenceRotCenter: List[Seq[Landmark[_3D]]],
                      files: List[File],
                      rotCenters: List[File]): TriangleMeshDataCollectionMultiObjects[_3D] = {
    val n = files(0).listFiles().size
    val m = files.size

    def testDataSize(files: List[File], rotCenters: List[File]): Boolean = {
      var j = 0

      for (i <- 0 to m - 1) {
        if (files(i).listFiles().size == n &&
            rotCenters(i).listFiles().size == n) {
          j = j + 1
        }
      }
      var bool: Boolean = true
      if (j == m) {
        bool = true
      } else {
        bool = false
      }
      bool

    }
    if (testDataSize(files, rotCenters)) {
      val data = for (i <- 0 to n - 1) yield {

        val multibody = for (j <- 0 to files.size - 1) yield {
          (MeshIO.readMesh(files(j).listFiles().sortBy(_.getName).apply(i)).get,
           LandmarkIO.readLandmarksJson[_3D](rotCenters(j).listFiles().sortBy(_.getName).apply(i)).get)
        }
        (multibody.map(d => d._1), multibody.map(d => d._2))

      }
      fromTriangleMesh3DSequence(referenceMesh,
                                 referenceRotCenter,
                                 data.map(d => d._1).toList,
                                 data.map(d => d._2).toList)
    } else {
      throw new Exception(
        "size of each object's dataset should be the same or" +
          "size of each object's dataset should be the same as number of rotcenters"
      )
    }
  }
  /*
   * Performs a Generalized Procrustes Analysis on the data collection.
   * This is done by repeatedly computing the mean of all MultiBodyObject[TriangleMesh] in the dataset and
   * aligning all items rigidly to the mean.
   *
   * The reference MultiBodyObject[TriangleMesh] is unchanged, only the transformations in the collection are adapted
   */
  def gpa(dc: TriangleMeshDataCollectionMultiObjects[_3D])(
    ): TriangleMeshDataCollectionMultiObjects[_3D] = {
    computeGPA(dc)
  }

  private def computeGPA(
    dc1: TriangleMeshDataCollectionMultiObjects[_3D]
  ): TriangleMeshDataCollectionMultiObjects[_3D] = {
    implicit val rng = scalismo.utils.Random(42)
    val items = dc1.dataItems
    val dataMeshes = for (i <- 0 to dc1.reference.objects.size - 1) yield {

      for (df <- items) yield {

        val warpFieldInterpolated = df.interpolate(NearestNeighborInterpolator())

        val restrictedDF = DiscreteField(df.domain.objects(i), warpFieldInterpolated.andThen(_.shapeVec))
        TriangleMesh.domainWarp3D.transformWithField(df.domain.objects(i), restrictedDF)

      }
    }

    val dcs = for (j <- 0 to dc1.reference.objects.size - 1) yield {
      val dc = DataCollection.fromTriangleMesh3DSequence(items(0).domain.objects(j), dataMeshes(j))
      DataCollection.gpa(dc)
    }

    val data = for (i <- 0 to dcs(0).size - 1) yield {

      val df = for (j <- 0 to dc1.reference.objects.size - 1) yield {
        val fieldInterpolated = items(i).interpolate(NearestNeighborInterpolator())
        val poseDF = DiscreteField(
          items(0).domain.objects(j),
          items(i).domain.objects(j).pointSet.points.toIndexedSeq.map(p => fieldInterpolated(p).poseVec)
        )

        (dcs(j),
         poseDF,
         fieldInterpolated(items(i).domain.rotationCenters(j)),
         fieldInterpolated(items(i).domain.neutralPoints(j)))
      }

      DiscreteField[_3D, MultiBodyObject[_3D, TriangleMesh]#DomainT, ShapeAndPoseVector[_3D]](
        items(0).domain,
        (for (j <- 0 to dc1.reference.objects.size - 1) yield {
          df(j)._1
            .dataItems(i)
            .pointsWithIds
            .map(pi => ShapeAndPoseVector(df(j)._1.dataItems(i).apply(pi._2), df(j)._2.apply(pi._2)))
            .toIndexedSeq ++
            IndexedSeq(df(j)._3) ++
            IndexedSeq(df(j)._4)
        }).reduce((acc, s) => acc ++ s)
      )
    }

    new DataCollection[_3D, MultiBodyObject[_3D, TriangleMesh]#DomainT, ShapeAndPoseVector[_3D]](data)
  }

}
