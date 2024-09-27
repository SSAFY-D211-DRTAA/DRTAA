plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.jetbrains.kotlin.android)
    alias(libs.plugins.gms)
    id("drtaa.plugin.feature")
}

android {
    namespace = "com.drtaa.feature_main"
    compileSdk = 34

    defaultConfig {
        minSdk = 28

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
    buildFeatures{
        dataBinding = true
        buildConfig = true
    }
    sourceSets {
        getByName("main") {
            res {
                srcDirs("src\\main\\res", "src\\main\\res")
            }
        }
    }
}

dependencies {
    // modules
    implementation(project(":feature-mypage"))
    implementation(project(":feature-home"))
    implementation(project(":feature-tour"))
    implementation(project(":feature-car"))
    implementation(project(":feature-rent"))
    implementation(project(":feature-payment"))
    implementation(project(":feature-plan"))
    implementation(libs.map.sdk)
    // Firebase
    implementation(platform(libs.firebase.bom))
    implementation(libs.firebase.messaging.ktx)
}