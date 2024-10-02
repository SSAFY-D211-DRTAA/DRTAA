package com.drtaa.feature_plan

import android.content.Intent
import android.net.Uri
import androidx.databinding.DataBindingUtil
import androidx.fragment.app.viewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.navArgs
import com.drtaa.core_model.plan.PlanItem
import com.drtaa.core_model.travel.Weather
import com.drtaa.core_ui.base.BaseFragment
import com.drtaa.core_ui.centerCrop
import com.drtaa.core_ui.loadImageUrl
import com.drtaa.feature_plan.adapter.PostListAdapter
import com.drtaa.feature_plan.databinding.FragmentTravelBinding
import com.drtaa.feature_plan.databinding.ItemTravelWeatherBinding
import com.drtaa.feature_plan.viewmodel.TravelViewModel
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import java.time.LocalDate
import java.time.format.DateTimeFormatter

@AndroidEntryPoint
class TravelFragment :
    BaseFragment<FragmentTravelBinding>(R.layout.fragment_travel) {

    private val travelViewModel: TravelViewModel by viewModels()

    private val args by navArgs<TravelFragmentArgs>()
    private val planItem: PlanItem by lazy { args.planItem }

    private lateinit var postListAdapter: PostListAdapter

    override fun initView() {
        val notice = LocalDate.now().format(DateTimeFormatter.ofPattern("*MM월 dd일 기준입니다."))
        binding.tvTravelWeatherHelp.text = notice
        binding.tvTravelTitle.text = planItem.datePlacesName
        binding.tvTravelAddress.text = planItem.datePlacesAddress

        initRVAdapter()
        initObserve()

        travelViewModel.getImage(planItem.datePlacesName)
        travelViewModel.getBlogPostList(planItem.datePlacesName)
        travelViewModel.getWeatherList(planItem.datePlacesLat, planItem.datePlacesLon)
    }

    private fun initObserve() {
        travelViewModel.mainImage.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { image ->
                if (image == null) {
                    binding.ivTravel.setImageResource(android.R.drawable.ic_menu_gallery)
                    return@onEach
                }

                binding.ivTravel.centerCrop(image.link, requireActivity())
            }.launchIn(viewLifecycleOwner.lifecycleScope)
        travelViewModel.postList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { postList ->
                if (postList == null) return@onEach

                postListAdapter.submitList(postList)
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        travelViewModel.weatherList.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { weatherList ->
                if (weatherList == null) return@onEach

                setWeatherView(weatherList)
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun setWeatherView(weatherList: List<Weather>) {
        binding.apply {
            val weatherViewList = (0 until clTravelWeather.childCount).map { index ->
                val view = clTravelWeather.getChildAt(index)
                DataBindingUtil.bind<ItemTravelWeatherBinding>(view)
            }

            weatherViewList.forEachIndexed { index, weatherView ->
                weatherView?.let {
                    weatherView.tvWeatherDay.text = weatherList[index].dayOfWeek
                    weatherView.tvWeatherMinTemp.text = "${weatherList[index].min.toInt()}°"
                    weatherView.tvWeatherMaxTemp.text = "${weatherList[index].max.toInt()}°"

                    val weatherStatus = weatherList[index].description.replace("d", "n")
                    weatherView.ivWeatherIcon.loadImageUrl("${WEATHER_IMG_URL + weatherStatus}.png")
                }
            }
        }
    }

    private fun initRVAdapter() {
        postListAdapter = PostListAdapter { post ->
            requireContext().startActivity(Intent(Intent.ACTION_VIEW, Uri.parse(post.link)))
        }
        binding.rvTravelBlog.adapter = postListAdapter
    }

    companion object {
        const val WEATHER_IMG_URL =
            "https://myd211s3bucket.s3.ap-northeast-2.amazonaws.com/weather/"
    }
}
