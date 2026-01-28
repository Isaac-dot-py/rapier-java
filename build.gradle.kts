import org.gradle.external.javadoc.StandardJavadocDocletOptions

group = "com.rapier"
version = "1.0.0"

plugins {
    `java-library`
    `maven-publish`
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))
    }
    withSourcesJar()
    withJavadocJar()
}

repositories {
    mavenCentral()
}

dependencies {
    api("net.java.dev.jna:jna:5.13.0")
    api("net.java.dev.jna:jna-platform:5.13.0")
}

val nativeOutputDir = layout.buildDirectory.dir("generated/resources/native")

val cargoBuild by tasks.registering(Exec::class) {
    group = "build"
    description = "Build the native Rust library via Cargo"
    workingDir = project.file("native-lib")
    environment("CARGO_TARGET_DIR", project.file("native-lib/target"))
    commandLine("cargo", "build", "--release")
}

val copyNative by tasks.registering(Copy::class) {
    group = "build"
    description = "Copy built native libraries into generated resources"
    dependsOn(cargoBuild)
    from("native-lib/target/release") {
        include("rapier_java_ffi.dll", "librapier_java_ffi.so", "librapier_java_ffi.dylib")
    }
    into(nativeOutputDir)
}

sourceSets {
    main {
        resources.srcDir(nativeOutputDir)
    }
}

tasks.named<ProcessResources>("processResources") { dependsOn(copyNative) }

tasks.named<Jar>("sourcesJar") { dependsOn(copyNative) }

tasks.withType<Javadoc>().configureEach {
    (options as? StandardJavadocDocletOptions)?.apply {
        addBooleanOption("Xdoclint:none", true)
    }
}

tasks.test {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            from(components["java"])
            pom {
                name.set("Rapier Java Bindings")
                description.set("Java 17 bindings for Rapier 2D physics engine (double precision)")
            }
        }
    }
    repositories {
        maven {
            name = "buildRepo"
            url = layout.buildDirectory.dir("m2").get().asFile.toURI()
        }
    }
}

// Utility task to emit runtime classpath for scripts
val printRuntimeClasspath by tasks.registering {
    group = "help"
    description = "Print the runtime classpath for the main source set"
    doLast {
        println(sourceSets.main.get().runtimeClasspath.asPath)
    }
}
