package ShapeAndPoseModels.io

import breeze.linalg.{DenseMatrix, DenseVector}
import ShapeAndPoseModels.{DomainWithPoseParameters, MultiBodyObject}
import ShapeAndPoseModels.ShapeAndPosePGA
import ncsa.hdf.`object`.Group
import scalismo.common.{DiscreteField, DomainWarp, PointId}
import scalismo.geometry.{_3D, EuclideanVector, Landmark, Point}
import scalismo.io._
import scalismo.mesh._
import scalismo.mesh.TriangleMesh.domainWarp3D
import java.io._
import java.util.Calendar

import scala.collection.parallel.immutable.ParSeq
import scala.util.{Failure, Success, Try}

object SingleBodyShapeAndPoseGPAIO {

  def readShapeAndPoseSingleBodyGPA(file: File): Try[ShapeAndPosePGA[TriangleMesh]] = {
    // currently, we support only the statismo format
    SingleBodyPGAIOImpl.readStatismoSingleBodyShapeAndPoseModel(file, "/")
  }

  /**
   * Writes a statistical multi meshmodel. The file type is determined
   * based on the extension. Currently on the Scalismo format (.h5)
   * is supported.
   *
   * @param model The statistical multi mesh model
   * @param file The file to which the model is written
   * @return In case of Failure, the Failure is returned.
   */
  def writeShapeAndPoseSingleBodyGPA(model: ShapeAndPosePGA[TriangleMesh], file: File): Try[Unit] = {
    // currently, we support only the statismo format
    SingleBodyPGAIOImpl.writeStatismoSingleBodyShapeAndPoseModel(model, file, "/")
  }

}

private object SingleBodyPGAIOImpl {

  //implicit val warper : DomainWarp[_3D, ({type T[D] = MultibodyObject[D, TriangleMesh]})#T] = ???
  //implicit val warper2 : DomainWarp[_3D, TriangleMesh] = ???

  object StatismoVersion extends Enumeration {
    type StatismoVersion = Value
    val v081, v090 = Value
  }

  import StatismoVersion._

  def writeStatismoSingleBodyShapeAndPoseModel(model: ShapeAndPosePGA[TriangleMesh],
                                               file: File,
                                               modelPath: String = "/",
                                               statismoVersion: StatismoVersion = v090): Try[Unit] = {

    //val df = DiscreteField[_3D, ({type T[D] = MultibodyObject[D, TriangleMesh]})#T, EuclideanVector[_3D]](model.gp.domain, model.gp.domain.pointSet.pointsWithId.toIndexedSeq.map(pt => model.mean.pointSet.point(pt._2) - pt._1))
    //val discretizedMean= model.logExpMapping.logMapping(df).data.map(v=>Array(v.shapeVec(0),v.shapeVec(1),v.shapeVec(2),v.poseVec(0),v.poseVec(1),v.poseVec(2))).reduce((acc,s)=>acc++s).toIndexedSeq
    val discretizedMean = model.gp.mean.data
      .map(v => Array(v.shapeVec(0), v.shapeVec(1), v.shapeVec(2), v.poseVec(0), v.poseVec(1), v.poseVec(2)))
      .flatten
    // val discretizedMean =  model.mean.pointSet.points.toIndexedSeq.flatten(_.toArray)
    val variance = model.gp.variance

    val pcaBasis = model.gp.basisMatrix.copy

    if (statismoVersion == v081) {
      // statismo 081 has the variance included in the pcaBasis
      for (i <- 0 until variance.length) {
        pcaBasis(::, i) *= math.sqrt(variance(i))
      }
    }
    val maybeError = for {
      h5file <- HDF5Utils.createFile(file)
      _ <- h5file.writeArray[Float](s"$modelPath/model/mean", discretizedMean.toArray.map(_.toFloat))
      _ <- h5file.writeArray[Float](s"$modelPath/model/noiseVariance", Array(0f))
      _ <- h5file.writeNDArray[Float](s"$modelPath/model/pcaBasis",
                                      NDArray(Array(pcaBasis.rows, pcaBasis.cols).map(_.toLong).toIndexedSeq,
                                              pcaBasis.t.flatten(false).toArray.map(_.toFloat)))
      _ <- h5file.writeArray[Float](s"$modelPath/model/pcaVariance", variance.toArray.map(_.toFloat))
      _ <- h5file.writeString(s"$modelPath/modelinfo/build-time", Calendar.getInstance.getTime.toString)
      group <- h5file.createGroup(s"$modelPath/representer")
      _ <- if (statismoVersion == v090) {
        for {
          _ <- writeRepresenterStatismov090_SingleBody(h5file, group, model, modelPath)
          _ <- h5file.writeInt("/version/majorVersion", 0)
          _ <- h5file.writeInt("/version/minorVersion", 9)
        } yield Success(())
      } else {
        for {
          _ <- writeRepresenterStatismov081_SingleBody(h5file, group, model, modelPath)
          _ <- h5file.writeInt("/version/majorVersion", 0)
          _ <- h5file.writeInt("/version/minorVersion", 8)
        } yield Success(())
      }
      _ <- h5file.writeString(s"$modelPath/modelinfo/modelBuilder-0/buildTime", Calendar.getInstance.getTime.toString)
      _ <- h5file.writeString(s"$modelPath/modelinfo/modelBuilder-0/builderName",
                              "This is a useless info. The stkCore did not handle Model builder info at creation time.")
      _ <- h5file.createGroup(s"$modelPath/modelinfo/modelBuilder-0/parameters")
      _ <- h5file.createGroup(s"$modelPath/modelinfo/modelBuilder-0/dataInfo")
      _ <- Try {
        h5file.close()
      }
    } yield ()

    maybeError
  }

  private def writeRepresenterStatismov090_SingleBody(h5file: HDF5File,
                                                      group: Group,
                                                      model: ShapeAndPosePGA[TriangleMesh],
                                                      modelPath: String): Try[Unit] = {
    Try {
      for {
        _ <- h5file.writeStringAttribute(group.getFullName, "name", "multiitkStandardMeshRepresenter")
        _ <- h5file.writeStringAttribute(group.getFullName, "version/majorVersion", "0")
        _ <- h5file.writeStringAttribute(group.getFullName, "version/minorVersion", "9")
        _ <- h5file.writeStringAttribute(group.getFullName, "datasetType", "POLYGON_MESH")
      } yield Success(())

      val cellArray1 = model.gp.domain.domain.cells.map(_.ptId1.id) ++
        model.gp.domain.domain.cells.map(_.ptId2.id) ++ model.gp.domain.domain.cells.map(_.ptId3.id)

      val pts1 = model.gp.domain.domain.pointSet.points.toIndexedSeq.map(p => (p.toArray(0), p.toArray(1), p.toArray(2))
      )

      val pointArray1 = pts1.map(_._1) ++ pts1.map(_._2) ++ pts1.map(_._3)

      val rotCenterPts = ParSeq(model.gp.domain.rotCenter.toArray(0),
                                model.gp.domain.rotCenter.toArray(1),
                                model.gp.domain.rotCenter.toArray(2))

      val neutralPts = ParSeq(model.gp.domain.neutralPoint.toArray(0),
                              model.gp.domain.neutralPoint.toArray(1),
                              model.gp.domain.neutralPoint.toArray(2))

      for {
        _ <- h5file.writeNDArray[Int](s"$modelPath/representer/cells",
                                      NDArray(IndexedSeq(3, model.gp.domain.domain.cells.size), cellArray1.toArray))
        _ <- h5file.writeNDArray[Float](
          s"$modelPath/representer/points",
          NDArray(IndexedSeq(3, model.gp.domain.domain.pointSet.points.size), pointArray1.toArray.map(_.toFloat))
        )

        _ <- h5file.writeNDArray[Float](s"$modelPath/representer/rotcenter",
                                        NDArray(IndexedSeq(3, 1.toInt), rotCenterPts.toArray.map(_.toFloat)))
        _ <- h5file.writeNDArray[Float](s"$modelPath/representer/neutralpoint",
                                        NDArray(IndexedSeq(3, 1.toInt), neutralPts.toArray.map(_.toFloat)))

      } yield Success(())

    }

  }

  private def writeRepresenterStatismov081_SingleBody(h5file: HDF5File,
                                                      group: Group,
                                                      model: ShapeAndPosePGA[TriangleMesh],
                                                      modelPath: String): Try[Unit] = {

    // we simply store the reference into a vtk file and store the file (the binary data) into the representer
    Try {
      def refAsByteArray(ref: TriangleMesh[_3D],
                         rotcenter: Point[_3D],
                         neutralpt: Point[_3D]): Try[(Array[Byte], Array[Byte], Array[Byte])] = {

        val tmpfile1 = File.createTempFile("temp", ".vtk")
        tmpfile1.deleteOnExit()

        val tmpfilerotcenter = File.createTempFile("temp", ".json")
        tmpfilerotcenter.deleteOnExit()

        val tmpfileneutralpts = File.createTempFile("temp", ".json")
        tmpfileneutralpts.deleteOnExit()

        for {
          _ <- MeshIO.writeMesh(ref, tmpfile1)
          _ <- LandmarkIO.writeLandmarksJson(Seq(Landmark[_3D](rotcenter.toVector.norm.toString, rotcenter)),
                                             tmpfilerotcenter)
          _ <- LandmarkIO.writeLandmarksJson(Seq(Landmark[_3D](neutralpt.toVector.norm.toString, neutralpt)),
                                             tmpfileneutralpts)
          ba <- readFileAsByteArray(tmpfile1, tmpfilerotcenter, tmpfileneutralpts)
        } yield ba

      }

      def readFileAsByteArray(f1: File,
                              frotcent: File,
                              fneutralpt: File): Try[(Array[Byte], Array[Byte], Array[Byte])] = {
        Try {
          val fileData1 = new Array[Byte](f1.length().toInt)
          val dis1 = new DataInputStream(new FileInputStream(f1))
          dis1.readFully(fileData1)
          dis1.close()

          val fileDatarotcent = new Array[Byte](frotcent.length().toInt)
          val disrotcent = new DataInputStream(new FileInputStream(frotcent))
          disrotcent.readFully(fileDatarotcent)
          disrotcent.close()

          val fileDataneutpt = new Array[Byte](fneutralpt.length().toInt)
          val disneutralpt = new DataInputStream(new FileInputStream(fneutralpt))
          disneutralpt.readFully(fileDataneutpt)
          disneutralpt.close()

          (fileData1, fileDatarotcent, fileDataneutpt)
        }
      }

      for {
        _ <- h5file.writeStringAttribute(group.getFullName, "name", "multiitkMeshRepresenter")
        ba <- refAsByteArray(model.gp.domain.domain, model.gp.domain.rotCenter, model.gp.domain.neutralPoint)
        _ <- h5file.writeNDArray[Byte](s"$modelPath/representer/reference", NDArray(IndexedSeq(ba._1.length, 1), ba._1))
        _ <- h5file.writeNDArray[Byte](s"$modelPath/representer/rotcenter", NDArray(IndexedSeq(ba._2.length, 1), ba._2))
        _ <- h5file.writeNDArray[Byte](s"$modelPath/representer/neutralpt", NDArray(IndexedSeq(ba._3.length, 1), ba._3))
      } yield Success(())

    }

  }

  private def ndFloatArrayToDoubleMatrix(array: NDArray[Float])(implicit dummy: DummyImplicit,
                                                                dummy2: DummyImplicit): DenseMatrix[Double] = {
    // the data in ndarray is stored row-major, but DenseMatrix stores it column major. We therefore
    // do switch dimensions and transpose
    DenseMatrix.create(array.dims(1).toInt, array.dims(0).toInt, array.data.map(_.toDouble)).t
  }

  private def ndIntArrayToIntMatrix(array: NDArray[Int]) = {
    // the data in ndarray is stored row-major, but DenseMatrix stores it column major. We therefore
    // do switch dimensions and transpose

    DenseMatrix.create(array.dims(1).toInt, array.dims(0).toInt, array.data).t
  }

  private def writeTmpFile(data: Array[Byte]): Try[File] = {
    val tmpfile = File.createTempFile("temp", ".vtk")
    tmpfile.deleteOnExit()

    Try {
      val stream = new DataOutputStream(new FileOutputStream(tmpfile))
      stream.write(data)
      stream.close()
    } map (_ => tmpfile)
  }

  private def writejsonTmpFile(data: Array[Byte]): Try[File] = {
    val tmpfile = File.createTempFile("temp", ".json")
    tmpfile.deleteOnExit()
    Try {
      val stream = new DataOutputStream(new FileOutputStream(tmpfile))
      stream.write(data)
      stream.close()
    } map (_ => tmpfile)
  }

  def readStatismoSingleBodyShapeAndPoseModel(file: File, modelPath: String = "/") = {

    def extractOrthonormalPCABasisMatrix(pcaBasisMatrix: DenseMatrix[Double],
                                         pcaVarianceVector: DenseVector[Double]): DenseMatrix[Double] = {
      // this is an old statismo format, that has the pcaVariance directly stored in the PCA matrix,
      // i.e. pcaBasis = U * sqrt(lambda), where U is a matrix of eigenvectors and lambda the corresponding eigenvalues.
      // We recover U from it.

      val lambdaSqrt = pcaVarianceVector.map(l => math.sqrt(l))
      val lambdaSqrtInv = lambdaSqrt.map(l => if (l > 1e-8) 1.0f / l else 0f)

      // The following code is an efficient way to compute: pcaBasisMatrix * breeze.linalg.diag(lambdaSqrtInv)
      // (diag returns densematrix, so the direct computation would be very slow)
      val U = DenseMatrix.zeros[Double](pcaBasisMatrix.rows, pcaBasisMatrix.cols)
      for (i <- 0 until pcaBasisMatrix.cols) {
        U(::, i) := pcaBasisMatrix(::, i) * lambdaSqrtInv(i)
      }
      U
    }

    val modelOrFailure = for {
      h5file <- HDF5Utils.openFileForReading(file)
      representerName <- h5file.readStringAttribute(s"$modelPath/representer/", "name")
      // read mesh according to type given in representer
      mesh <- representerName match {
        case "multivtkPolyDataRepresenter" => readVTKSingleBodyShapeAndPoseFromRepresenterGroup(h5file, modelPath)
        case "multiitkMeshRepresenter"     => readVTKSingleBodyShapeAndPoseFromRepresenterGroup(h5file, modelPath)
        case _ =>
          h5file.readStringAttribute(s"$modelPath/representer/", "datasetType") match {
            case Success("POLYGON_MESH") => readStandardSingleBodyShapeAndPoseFromRepresenterGroup(h5file, modelPath)
            case Success(datasetType) =>
              Failure(new Exception(s"can only read model of datasetType POLYGON_MESH. Got $datasetType instead"))
            case Failure(t) => Failure(t)
          }
      }

      meanArray <- h5file.readNDArray[Float](s"$modelPath/model/mean")
      meanVector = DenseVector(meanArray.data.map(_.toDouble))
      pcaBasisArray <- h5file.readNDArray[Float](s"$modelPath/model/pcaBasis")
      majorVersion <- if (h5file.exists("/version/majorVersion")) h5file.readInt("/version/majorVersion")
      else {
        if (representerName == "multivtkPolyDataRepresenter" || representerName == "multiitkMeshRepresenter") Success(0)
        else Failure(new Throwable(s"no entry /version/majorVersion provided in statismo file"))
      }
      minorVersion <- if (h5file.exists("/version/minorVersion")) h5file.readInt("/version/minorVersion")
      else {
        if (representerName == "multivtkPolyDataRepresenter" || representerName == "multiitkMeshRepresenter") Success(8)
        else Failure(new Throwable(s"no entry /version/minorVersion provided in statismo file."))
      }
      pcaVarianceArray <- h5file.readNDArray[Float](s"$modelPath/model/pcaVariance")
      pcaVarianceVector = DenseVector(pcaVarianceArray.data.map(_.toDouble))
      pcaBasisMatrix = ndFloatArrayToDoubleMatrix(pcaBasisArray)
      pcaBasis <- (majorVersion, minorVersion) match {
        case (1, _) => Success(pcaBasisMatrix)
        case (0, 9) => Success(pcaBasisMatrix)
        case (0, 8) =>
          Success(extractOrthonormalPCABasisMatrix(pcaBasisMatrix, pcaVarianceVector)) // an old statismo version
        case v => Failure(new Throwable(s"Unsupported version ${v._1}.${v._2}"))
      }

      _ <- Try {
        h5file.close()
      }
    } yield {
      // statismo stores the mean as the point position, not as a displacement on the reference.
      def flatten(v: IndexedSeq[Point[_3D]]) = DenseVector(v.flatten(pt => Array(pt(0), pt(1), pt(2))).toArray)

      //      val refpointsVec = flatten(mesh.pointSet.points.toIndexedSeq)
      //      val meanDefVector = meanVector - refpointsVec

      ShapeAndPosePGA(mesh, meanVector, pcaVarianceVector, pcaBasis)
    }

    modelOrFailure
  }

  private def readStandardSingleBodyShapeAndPoseFromRepresenterGroup(
    h5file: HDF5File,
    modelPath: String
  ): Try[DomainWithPoseParameters[_3D, TriangleMesh]] = {

    val mesh1 = for {
      vertArray1 <- h5file
        .readNDArray[Float](s"$modelPath/representer/points")
        .flatMap(vertArray =>
          if (vertArray.dims(0) != 3)
            Failure(new Exception("the representer points1 are not 3D points"))
          else
            Success(vertArray)
        )
      vertMat = ndFloatArrayToDoubleMatrix(vertArray1)
      points = for (i <- 0 until vertMat.cols) yield Point(vertMat(0, i), vertMat(1, i), vertMat(2, i))
      cellArray <- h5file
        .readNDArray[Int](s"$modelPath/representer/cells")
        .flatMap(cellArray =>
          if (cellArray.dims(0) != 3)
            Failure(new Exception("the representer cells1 are not triangles"))
          else
            Success(cellArray)
        )
      cellMat = ndIntArrayToIntMatrix(cellArray)
      cells = for (i <- 0 until cellMat.cols)
        yield TriangleCell(PointId(cellMat(0, i)), PointId(cellMat(1, i)), PointId(cellMat(2, i)))
      cellArray <- h5file.readNDArray[Int](s"$modelPath/representer/cells")
    } yield TriangleMesh3D(points, TriangleList(cells))

    val rotcenters = for {
      vertArray <- h5file
        .readNDArray[Float](s"$modelPath/representer/rotcenter")
        .flatMap(vertArray =>
          if (vertArray.dims(0) != 3)
            Failure(new Exception("the representer points are not 3D points"))
          else
            Success(vertArray)
        )
      vertMat = ndFloatArrayToDoubleMatrix(vertArray)
      points = for (i <- 0 until vertMat.cols) yield Point(vertMat(0, i), vertMat(1, i), vertMat(2, i))
    } yield points

    val neutralpts = for {
      vertArray <- h5file
        .readNDArray[Float](s"$modelPath/representer/neutralpoint")
        .flatMap(vertArray =>
          if (vertArray.dims(0) != 3)
            Failure(new Exception("the representer points are not 3D points"))
          else
            Success(vertArray)
        )
      vertMat = ndFloatArrayToDoubleMatrix(vertArray)
      points = for (i <- 0 until vertMat.cols) yield Point(vertMat(0, i), vertMat(1, i), vertMat(2, i))
    } yield points

    (mesh1.get, rotcenters.get, neutralpts.get)

    Try(DomainWithPoseParameters(mesh1.get, rotcenters.get(0), neutralpts.get(0)))
  }

  //  /*
  //   * reads the reference (a vtk file), which is stored as a byte array in the hdf5 file)
  //  */
  //  private def readVTKMeshFromRepresenterGroup(h5file: HDF5File, modelPath: String,i:Int): Try[TriangleMesh[_3D]] = {
  //
  //
  //    for {
  //      rawdata <- h5file.readNDArray[Byte](s"$modelPath/representer/reference"+i)
  //      vtkFile <- writeTmpFile(rawdata.data)
  //      triangleMesh <- MeshIO.readMesh(vtkFile)
  //    } yield triangleMesh
  //
  //  }
  //

  /*
   * reads the joint reference (a vtk file), which is stored as a byte array in the hdf5 file)
   */
  private def readVTKSingleBodyShapeAndPoseFromRepresenterGroup(
    h5file: HDF5File,
    modelPath: String
  ): Try[DomainWithPoseParameters[_3D, TriangleMesh]] = {

    //    val numberObjects=for {
    //      numberobject <- h5file.readNDArray[Float](s"$modelPath/model/numberObjects")
    //
    //    } yield numberobject

    val mesh1 = for {
      rawdata <- h5file.readNDArray[Byte](s"$modelPath/representer/reference")
      vtkFile <- writeTmpFile(rawdata.data)
      triangleMesh <- MeshIO.readMesh(vtkFile)
    } yield triangleMesh

    val rotcenters = for {
      rawdata <- h5file.readNDArray[Byte](s"$modelPath/representer/rotcenter")
      vtkFile <- writejsonTmpFile(rawdata.data)
      landmark <- LandmarkIO.readLandmarksJson[_3D](vtkFile)
    } yield landmark.map(l => l.point).toList

    val neutralpts = for {
      rawdata <- h5file.readNDArray[Byte](s"$modelPath/representer/neutralpoint")
      vtkFile <- writejsonTmpFile(rawdata.data)
      landmark <- LandmarkIO.readLandmarksJson[_3D](vtkFile)
    } yield landmark.map(l => l.point).toList

    Try(DomainWithPoseParameters(mesh1.get, rotcenters.get(0), neutralpts.get(0)))
  }

}
