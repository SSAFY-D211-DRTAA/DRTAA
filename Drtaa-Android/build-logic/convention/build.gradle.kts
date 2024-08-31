import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    `kotlin-dsl`
}

group = "com.drtaa.convention"

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

kotlin {
    compilerOptions {
        jvmTarget = JvmTarget.JVM_17
    }
}

dependencies {
    compileOnly(libs.android.gradlePlugin)
    compileOnly(libs.kotlin.gradlePlugin)
}

tasks {
    validatePlugins {
        enableStricterValidation = true
        failOnWarning = true
    }
}

gradlePlugin {
    plugins {
        register("AndroidHiltPlugin") {
            id = "drtaa.plugin.hilt"
            implementationClass = "AndroidHiltConventionPlugin"
        }
        register("AndroidCorePlugin") {
            id = "drtaa.plugin.core"
            implementationClass = "AndroidCoreConventionPlugin"
        }
        register("AndroidCommonPlugin") {
            id = "drtaa.plugin.common"
            implementationClass = "AndroidCommonConventionPlugin"
        }
        register("AndroidCoreNetworkPlugin") {
            id = "drtaa.plugin.network"
            implementationClass = "AndroidCoreNetworkConventionPlugin"
        }
        register("AndroidFeaturePlugin") {
            id = "drtaa.plugin.feature"
            implementationClass = "AndroidFeatureConventionPlugin"
        }
    }
}