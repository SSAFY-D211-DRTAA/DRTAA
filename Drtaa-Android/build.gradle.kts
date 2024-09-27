// Top-level build file where you can add configuration options common to all sub-projects/modules.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.jetbrains.kotlin.android) apply false
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.hilt.gradle.plugin) apply false
    alias(libs.plugins.kotlin.kapt) apply false
    alias(libs.plugins.kotlinx.serialization) apply false
    alias(libs.plugins.navigation.safe.args) apply false
    alias(libs.plugins.jetbrains.kotlin.jvm) apply false
    alias(libs.plugins.detekt) apply false
    alias(libs.plugins.gms) apply false
}

apply(from = "gradle/projectDependencyGraph.gradle")
apply(plugin = "io.gitlab.arturbosch.detekt")

val reportMerge by tasks.registering(io.gitlab.arturbosch.detekt.report.ReportMergeTask::class) {
    output.set(layout.buildDirectory.file("reports/detekt/detekt.sarif"))
}

subprojects {
    apply(plugin = "io.gitlab.arturbosch.detekt")

    plugins.withType<io.gitlab.arturbosch.detekt.DetektPlugin>().configureEach {
        tasks.withType<io.gitlab.arturbosch.detekt.Detekt>().configureEach detekt@{
            reports {
                md.required.set(true)
                html.required.set(true)
            }

            finalizedBy(reportMerge)

            reportMerge.configure {
                input.from(this@detekt.sarifReportFile)
            }
        }
    }
}