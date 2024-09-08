import com.android.build.gradle.internal.cxx.configure.gradleLocalProperties

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.jetbrains.kotlin.android)
    id("drtaa.plugin.detekt")
    id("drtaa.plugin.network")
    id("drtaa.plugin.common")
    id("drtaa.plugin.hilt")
}

fun getApiKey(propertyKey: String): String {
    return gradleLocalProperties(rootDir, providers).getProperty(propertyKey)
}

android {
    namespace = "com.drtaa.android"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.drtaa.android"
        minSdk = 28
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        buildConfigField("String", "NAVER_MAP_CLIENT_ID", getApiKey("NAVER_MAP_CLIENT_ID"))
        buildConfigField("String", "NAVER_MAP_CLIENT_SECRET", getApiKey("NAVER_MAP_CLIENT_SECRET"))
        buildConfigField("String", "NAVER_LOGIN_CLIENT_ID", getApiKey("NAVER_LOGIN_CLIENT_ID"))
        buildConfigField("String", "NAVER_LOGIN_CLIENT_SECRET", getApiKey("NAVER_LOGIN_CLIENT_SECRET"))

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        manifestPlaceholders["NAVER_MAP_CLIENT_ID_MANIFEST"] = getApiKey("NAVER_MAP_CLIENT_ID_MANIFEST")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
    buildFeatures {
        dataBinding = true
        buildConfig = true
    }
}

dependencies {
    // modules
    implementation(project(":core-data"))
    implementation(project(":core-network"))
    implementation(project(":core-ui"))
    implementation(project(":core-model"))
    implementation(project(":feature-main"))
    implementation(project(":feature-ticket"))
    implementation(project(":feature-mypage"))
    implementation(project(":feature-home"))
    implementation(project(":feature-sign"))

    // Datastore
    implementation(libs.datastore.preferences)
    // Lifecycle
    implementation(libs.lifecycle.runtime.ktx)
    implementation(libs.lifecycle.extensions)
    // Coroutine
    implementation(libs.coroutines.android)
    implementation(libs.coroutines.core)
    //Sign
    implementation(libs.naver.oauth)
}