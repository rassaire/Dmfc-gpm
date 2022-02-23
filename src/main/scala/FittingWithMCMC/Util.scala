package FittingWithMCMC

import java.io.File

import ShapeAndPoseModels.MultiBodyShapeAndPosePGA
import breeze.linalg.DenseVector
import scalismo.common.DiscreteField.ScalarVolumeMeshField
import scalismo.common.{DiscreteDomain, _}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.io.MeshIO
import scalismo.mesh
import scalismo.mesh.boundingSpheres.Tetrahedron
import scalismo.mesh._


object Util {
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

  def averageHausdorffDistance(m1: TriangleMesh[_3D], m2: TriangleMesh[_3D]): Double = {
    def allDistsBetweenMeshes(mm1: TriangleMesh[_3D], mm2: TriangleMesh[_3D]): Iterator[Double] = {
      for (ptM1 <- mm1.pointSet.points) yield {
        val cpM2 = mm2.operations.closestPointOnSurface(ptM1).point
        (ptM1 - cpM2).norm
      }
    }

    val d1 = allDistsBetweenMeshes(m1, m2)

    print("sum=", d1.sum)
    print("size=", d1.toIndexedSeq.size)

    (d1.sum)*1.0/d1.size
  }




  def mergeMeshes(m:List[TriangleMesh[_3D]]):TriangleMesh[_3D]={
    var n=m(0).pointSet.numberOfPoints


    val seq=(for (i<- 0 to m.size-1) yield {
      m(i).pointSet.points
    }).reduce((acc,s)=>acc++s)

    val listCells= (for (i<- 1 to m.size-1) yield {
      val listCelli = m(i).triangulation.triangles.map(cell => TriangleCell(PointId(cell.ptId1.id + n),
        PointId(cell.ptId2.id + n),
        PointId(cell.ptId3.id + n)))
      n=n+m(i).pointSet.numberOfPoints
      listCelli
    }).reduce((acc,s)=>acc++s)

    val listcell=m(0).triangulation.triangles++listCells


    TriangleMesh3D(seq.toIndexedSeq,TriangleList(listcell))

  }

  def mergeScalarVolumes(m:List[DiscreteField[_3D, TetrahedralMesh, Double]]):ScalarVolumeMeshField[Double]={
    var n=m(0).domain.pointSet.numberOfPoints


    val seq=(for (i<- 0 to m.size-1) yield {
      m(i).domain.pointSet.points
    }).reduce((acc,s)=>acc++s)

    val data=(for (i<- 0 to m.size-1) yield {
      m(i).values
    }).reduce((acc,s)=>acc++s)

    val listCells= (for (i<- 1 to m.size-1) yield {
      val listCelli = m(i).domain.tetrahedralization.tetrahedrons.map(cell => TetrahedralCell(PointId(cell.ptId1.id + n),
        PointId(cell.ptId2.id + n),
        PointId(cell.ptId3.id + n),
        PointId(cell.ptId3.id + n)))
      n=n+m(i).domain.pointSet.numberOfPoints
      listCelli
    }).reduce((acc,s)=>acc++s)

    val listcell=m(0).domain.tetrahedralization.tetrahedrons++listCells

    DiscreteField3D(TetrahedralMesh3D(seq.toIndexedSeq,TetrahedralList(listcell)), data.toIndexedSeq)

  }

  def splitMeshes(m: TriangleMesh[_3D],numberOfMeshes:Int,  sizesCells:List[Int], sizesPts:List[Int]):List[TriangleMesh[_3D]]={
    var countpts=0
    var countCells=0
    //println("start loop ...")
    val meshes =for (i<- 0 until numberOfMeshes) yield {

      val ptsi=m.pointSet.points.toIndexedSeq.slice(countpts,countpts+sizesPts(i))
      //println(ptsi.size)
      val cellsi=m.triangulation.triangles.slice(countCells, countCells+sizesCells(i))
      val cells=cellsi.map(cell=>TriangleCell(PointId(cell.ptId1.id-countpts),PointId(cell.ptId2.id-countpts),PointId(cell.ptId3.id-countpts)))
      //println(cellsi.size)

     // println("countpts before"+i, countpts)
      countpts=countpts+sizesPts(i)
      //println("countpts after "+i, countpts)
      //println("countCells before "+i, countCells)
      countCells =countCells+sizesCells(i)
      //println("countcells after"+i, countCells)

      TriangleMesh3D(ptsi,TriangleList(cells))
    }
    meshes.toList
  }
  def sampling(ShapeAndPosePGA: MultiBodyShapeAndPosePGA[TriangleMesh],n_pc:Int, max_coef: Int, path:String,
               sizeCells1: List[Int], sizeCells2: List[Int],
               sizepts1: List[Int],sizePts2: List[Int]):Unit={
    var v=DenseVector.zeros[Double](ShapeAndPosePGA.gp.rank)
    var count=0
    val coefs =for (param <- 0 until 2 * max_coef-1) yield{

      if (param < max_coef){
        param

      }else{
        count = count + 1

        -count

      }


    }
    //print(coefs)

    for (coef <- coefs){
      v(n_pc)=coef

      val sample=ShapeAndPosePGA.instance(v).objects

      val femurSplit=splitMeshes(sample.head, 2,sizeCells1,sizepts1)
      val tibiaSplit=splitMeshes(sample.last,2,sizeCells2,sizePts2)


      //val sample= mergeMeshes(ShapeAndPosePGA.instance(v).objects) //the use of mergeMeshes may not be necessary, it is just use for combining several meshes into one.
      MeshIO.writeMesh(femurSplit.head, new File (path+"/PC"+n_pc+"/femur"+coef+".stl"))
      MeshIO.writeMesh(femurSplit.last, new File (path+"/PC"+n_pc+"/femur-cartilage"+coef+".stl"))
      MeshIO.writeMesh(tibiaSplit.head, new File (path+"/PC"+n_pc+"/tibia"+coef+".stl"))
      MeshIO.writeMesh(tibiaSplit.last, new File (path+"/PC"+n_pc+"/tibia-cartilage"+coef+".stl"))

     List(femurSplit.head, femurSplit.last, tibiaSplit.head, tibiaSplit.last)
    }







  }

  def corr(land1:Seq[Landmark[_3D]],
           land2:Seq[Landmark[_3D]],
           targetMesh: TriangleMesh[_3D],
           refMesh: TriangleMesh[_3D],
           cartilage: TriangleMesh[_3D])={


    val seqpts1=land1.map(l=>l.point).toIndexedSeq

    val DF1=seqpts1.map(p=>refMesh.pointSet.findClosestPoint(p).point-p).toIndexedSeq
    val seqpt2 = land2.map(l=>l.point).toIndexedSeq
    val DF2= seqpt2.map(p=>targetMesh.pointSet.findClosestPoint(p).point-p).toIndexedSeq
    //val d=UnstructuredPointsDomain3D((seqpts1++seqpt2).toIndexedSeq)
    val d=UnstructuredPointsDomain3D(seqpts1++seqpt2)

    val df= DiscreteField(d,DF1++DF2).interpolate(NearestNeighborInterpolator())

    def t(p:Point[_3D])={
      p+df(p)
    }
    cartilage.transform(t)

  }
}