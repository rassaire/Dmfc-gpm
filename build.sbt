import sbt.Resolver
import sbt.Keys._
import sbtassembly.AssemblyPlugin.autoImport.assemblyMergeStrategy


lazy val root = project
  .in(file("."))
  .settings(
    name := "DMFC-GPM-tutorial",
    version := "0.1",
    scalaVersion := "2.13.4",

    resolvers ++= Seq(

      Opts.resolver.sonatypeSnapshots,

      "shapmodelling-dmi" at "https://shapemodelling.cs.unibas.ch/repo"

    ),

      libraryDependencies  ++= Seq(

      "ch.unibas.cs.gravis" % "scalismo-native-all" % "4.0.+",

      "ch.unibas.cs.gravis" %% "scalismo-ui" % "0.90.0",

      "io.github.cibotech" %% "evilplot" % "0.8.1",

      "latim" %% "dmfc-gpm" % "0.1"

    ),
    assemblyJarName in assembly := "dmfc-gpm.jar",

    mainClass in assembly := Some("Example.multiObjectsModels"),

    assemblyMergeStrategy in assembly := {
      case PathList("META-INF", "MANIFEST.MF") => MergeStrategy.discard
      case PathList("META-INF", s) if s.endsWith(".SF") || s.endsWith(".DSA") || s.endsWith(".RSA") =>
        MergeStrategy.discard
      case _ => MergeStrategy.first
    }


  )

//organization := "uct.ac.za"//& latim.univ-brest.fr & imt-atlantique.fr

//name := "DMFC-GPM"
//
//version := "0.1"
//
//scalaVersion := "2.13.4"
//
//publishMavenStyle := true
//
//publishTo := Some(Resolver.file("file", new File("/Users/Jean-Rassaire/.ivy2/local")))
//
//scalacOptions := Seq("-unchecked", "-deprecation", "-encoding", "utf8")
//
//resolvers ++= Seq(
//  Resolver.bintrayRepo("unibas-gravis", "maven"),
//  Resolver.bintrayRepo("cibotech", "public"),
//  Opts.resolver.sonatypeSnapshots
//)
//
//libraryDependencies ++= Seq(
//  "ch.unibas.cs.gravis" % "scalismo-native-all" % "4.0.+",
//  "ch.unibas.cs.gravis" %% "scalismo-ui" % "0.90.0"
//)
//
//assemblyJarName in assembly := "dmfc-gpm.jar"
//
//mainClass in assembly := Some("Example.multiObjectsModels")
//
//assemblyMergeStrategy in assembly := {
//  case PathList("META-INF", "MANIFEST.MF")                                                      => MergeStrategy.discard
//  case PathList("META-INF", s) if s.endsWith(".SF") || s.endsWith(".DSA") || s.endsWith(".RSA") => MergeStrategy.discard
//  case "reference.conf"                                                                         => MergeStrategy.concat
//  case _                                                                                        => MergeStrategy.first
//}
