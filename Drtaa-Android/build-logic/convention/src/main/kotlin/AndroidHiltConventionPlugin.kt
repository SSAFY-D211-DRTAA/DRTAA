import extenstion.libs
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.kotlin.dsl.apply
import org.gradle.kotlin.dsl.dependencies

internal class AndroidHiltConventionPlugin : Plugin<Project> {
    override fun apply(target: Project) {
        with(target) {
            with(pluginManager){
                apply("com.google.dagger.hilt.android")
                apply("org.jetbrains.kotlin.kapt")
            }

            dependencies {
                add("kapt", libs.findLibrary("hilt.compiler").get())
                add("kapt", libs.findLibrary("androidx.hilt.compiler").get())
                add("implementation", libs.findLibrary("hilt.android").get())
            }
        }
    }
}