import extenstion.libs
import io.gitlab.arturbosch.detekt.Detekt
import io.gitlab.arturbosch.detekt.DetektCreateBaselineTask
import io.gitlab.arturbosch.detekt.extensions.DetektExtension
import org.gradle.api.Plugin
import org.gradle.kotlin.dsl.dependencies
import org.gradle.api.Project
import org.gradle.kotlin.dsl.configure
import org.gradle.kotlin.dsl.withType

class DetektConventionPlugin : Plugin<Project> {
    override fun apply(target: Project) {
        with(target) {
            pluginManager.apply("io.gitlab.arturbosch.detekt")

            dependencies {
                add("detektPlugins", libs.findLibrary("detekt.formatting").get())
            }

            extensions.configure<DetektExtension> {
                toolVersion = "1.23.5"
                config.setFrom(files("${rootProject.projectDir}/config/detekt.yml"))
                buildUponDefaultConfig = true
                parallel = true
            }

            tasks.withType<Detekt>().configureEach {
                reports {
                    html.required.set(true)
                    xml.required.set(true)
                    txt.required.set(false)
                    sarif.required.set(true)
                }
                jvmTarget = "17"
            }

            tasks.withType<DetektCreateBaselineTask>().configureEach {
                jvmTarget = "17"
            }
        }
    }
}