pluginManagement {
    includeBuild("build-logic")
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        maven("https://jitpack.io")
        maven("https://repository.map.naver.com/archive/maven")
    }
}

rootProject.name = "DRTAA"

include(":app")
include(":core-data")
include(":core-network")
include(":feature-main")
include(":core-ui")
include(":core-model")
include(":feature-tour")
include(":feature-mypage")
include(":feature-home")
include(":feature-sign")
include(":feature-car")
include(":core-map")
include(":feature-rent")
include(":core-mqtt")
include(":feature-payment")
include(":feature-plan")
