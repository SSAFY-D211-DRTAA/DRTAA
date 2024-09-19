import extenstion.libs
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.kotlin.dsl.dependencies

internal class AndroidCoreConventionPlugin : Plugin<Project> {
    override fun apply(target: Project) {
        with(target) {
            with(pluginManager) {
                apply("drtaa.plugin.network")
            }

            dependencies {
                add("implementation", project(":core-network"))
                add("implementation", libs.findLibrary("coroutines.android").get())
                add("implementation", libs.findLibrary("coroutines.core").get())
                add("implementation", libs.findLibrary("datastore.preferences").get())
            }
        }
    }
}