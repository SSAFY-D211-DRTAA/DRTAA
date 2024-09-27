import com.android.build.gradle.internal.cxx.configure.gradleLocalProperties

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.jetbrains.kotlin.android)
    id("drtaa.plugin.network")
}

fun getApiKey(propertyKey: String): String {
    return gradleLocalProperties(rootDir, providers).getProperty(propertyKey)
}

android {
    namespace = "com.drtaa.core_network"
    compileSdk = 34

    defaultConfig {
        minSdk = 28

        buildConfigField("String", "NAVER_MAP_CLIENT_ID", getApiKey("NAVER_MAP_CLIENT_ID"))
        buildConfigField("String", "NAVER_MAP_CLIENT_SECRET", getApiKey("NAVER_MAP_CLIENT_SECRET"))
        buildConfigField("String", "NAVER_CLIENT_ID", getApiKey("NAVER_CLIENT_ID"))
        buildConfigField("String", "NAVER_CLIENT_SECRET", getApiKey("NAVER_CLIENT_SECRET"))
        buildConfigField("String", "BASE_URL", getApiKey("BASE_URL"))

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
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
        buildConfig = true
    }
}

dependencies {
    implementation(libs.gson)
}