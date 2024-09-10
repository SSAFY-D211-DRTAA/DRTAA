import extenstion.libs
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.kotlin.dsl.dependencies

internal class AndroidCoreNetworkConventionPlugin : Plugin<Project> {
    override fun apply(target: Project) {
        with(target) {
            with(pluginManager) {
                apply("drtaa.plugin.hilt")
                apply("drtaa.plugin.common")
            }

            dependencies {
                add("implementation", project(":core-model"))
                add("implementation", libs.findLibrary("androidx.datastore.preferences.core").get())
                add("implementation", libs.findLibrary("okhttp").get())
                add("implementation", platform(libs.findLibrary("okhttp.bom").get()))
                add("implementation", libs.findLibrary("okhttp.loggingInterceptor").get())
                add("implementation", libs.findLibrary("retrofit").get())
                add("implementation", libs.findLibrary("retrofit.converter.gson").get())
                add("implementation", libs.findLibrary("retrofit.converter.kotlinxSerialization").get())
                add("implementation", libs.findLibrary("retrofit.converter.sclars").get())
                add("implementation", libs.findLibrary("kotlinx.serialization.core").get())
                add("implementation", libs.findLibrary("kotlinx.serialization.json").get())
            }
        }
    }
}