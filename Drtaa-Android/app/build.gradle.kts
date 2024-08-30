plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.jetbrains.kotlin.android)

    alias(libs.plugins.hilt.gradle.plugin)
    alias(libs.plugins.kotlin.kapt)
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

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
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
    buildFeatures{
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

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)

    // Datastore
    implementation(libs.datastore.preferences)
    // Lifecycle
    implementation(libs.lifecycle.runtime.ktx)
    implementation(libs.lifecycle.extensions)
    // Logging
    implementation(libs.timber)
    // Coroutine
    implementation(libs.coroutines.android)
    implementation(libs.coroutines.core)
    // DI
    implementation(libs.hilt.android)
    kapt(libs.hilt.compiler)
    kapt(libs.androidx.hilt.compiler)
}