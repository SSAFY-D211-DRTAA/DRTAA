plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.jetbrains.kotlin.android)
    id("drtaa.plugin.common")
    id("drtaa.plugin.hilt")
}

android {
    namespace = "com.drtaa.core_ui"
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
    }
}

dependencies {
    // Navigation
    implementation(libs.navigation.ui.ktx)
    implementation(libs.navigation.fragment.ktx)
    // Datastore
    implementation(libs.datastore.preferences)
    // Lifecycle
    implementation(libs.lifecycle.runtime.ktx)
    implementation(libs.lifecycle.extensions)
    // Media
    implementation(libs.glide)
    // Coroutine
    implementation(libs.coroutines.android)
    implementation(libs.coroutines.core)
    // DI
    implementation(libs.androidx.hilt.navigation.fragment)
    implementation(libs.play.services.location)
    //Calendar
    implementation("com.github.prolificinteractive:material-calendarview:2.0.1")
}