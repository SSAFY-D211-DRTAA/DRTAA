import extenstion.libs
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.kotlin.dsl.dependencies

internal class AndroidFeatureConventionPlugin : Plugin<Project> {

    override fun apply(target: Project) {
        with(target) {
            with(pluginManager) {
                apply("drtaa.plugin.common")
                apply("drtaa.plugin.hilt")
                apply(libs.findPlugin("navigation.safe.args").get().get().pluginId)
            }

            dependencies {
                add("implementation", project(":core-data"))
                add("implementation", project(":core-ui"))
                add("implementation", project(":core-model"))
                add("implementation", libs.findLibrary("navigation.ui.ktx").get())
                add("implementation", libs.findLibrary("navigation.fragment.ktx").get())
                add("implementation", libs.findLibrary("androidx.hilt.navigation.fragment").get())
                add("implementation", libs.findLibrary("datastore.preferences").get())
                add("implementation", libs.findLibrary("glide").get())
                add("implementation", libs.findLibrary("lifecycle.runtime.ktx").get())
                add("implementation", libs.findLibrary("lifecycle.extensions").get())
                add("implementation", libs.findLibrary("coroutines.android").get())
                add("implementation", libs.findLibrary("coroutines.core").get())
            }
        }
    }
}