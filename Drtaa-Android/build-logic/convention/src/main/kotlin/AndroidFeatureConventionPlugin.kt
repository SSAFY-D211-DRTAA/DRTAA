import extenstion.libs
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.kotlin.dsl.dependencies

internal class AndroidFeatureConventionPlugin : Plugin<Project> {

    override fun apply(target: Project) {
        with(target) {
            with(pluginManager) {
                apply("drtaa.plugin.hilt")
                apply("androidx.navigation.safeargs.kotlin")
            }

            dependencies {
                add("implementation", project(":core-data"))
                add("implementation", project(":core-ui"))
                add("implementation", project(":core-model"))
                add("implementation", libs.findLibrary("androidx.core.ktx").get())
                add("implementation", libs.findLibrary("androidx.appcompat").get())
                add("implementation", libs.findLibrary("material").get())
                add("implementation", libs.findLibrary("junit").get())
                add("androidTestImplementation", libs.findLibrary("androidx.junit").get())
                add("androidTestImplementation", libs.findLibrary("androidx.espresso.core").get())
                add("implementation", libs.findLibrary("navigation.ui.ktx").get())
                add("implementation", libs.findLibrary("navigation.fragment.ktx").get())
                add("implementation", libs.findLibrary("datastore.preferences").get())
                add("implementation", libs.findLibrary("timber").get())
                add("implementation", libs.findLibrary("glide").get())
                add("implementation", libs.findLibrary("lifecycle.runtime.ktx").get())
                add("implementation", libs.findLibrary("lifecycle.extensions").get())
                add("implementation", libs.findLibrary("coroutines.android").get())
                add("implementation", libs.findLibrary("coroutines.core").get())
                add("implementation", libs.findLibrary("androidx.appcompat").get())
            }
        }
    }
}