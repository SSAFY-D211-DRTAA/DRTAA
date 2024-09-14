package com.drtaa.feature_rent

import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import com.drtaa.core_ui.base.BaseBottomSheetDialogFragment
import com.drtaa.feature_rent.databinding.FragmentCalendarBottomSheetBinding
import com.drtaa.feature_rent.viewmodel.RentViewModel
import com.prolificinteractive.materialcalendarview.CalendarDay
import com.prolificinteractive.materialcalendarview.format.ArrayWeekDayFormatter
import org.threeten.bp.format.DateTimeFormatter
import timber.log.Timber

class CalendarBottomSheetDialogFragment :
    BaseBottomSheetDialogFragment<FragmentCalendarBottomSheetBinding>(R.layout.fragment_calendar_bottom_sheet) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)

    private var selectedStartDate = ""
    private var selectedEndDate = ""

    private lateinit var dayDecorator: DayDecorator
    private lateinit var middleDayDecorator: MiddleDayDecorator
    private lateinit var todayDecorator: TodayDecorator
    private lateinit var sundayDecorator: SundayDecorator
    private lateinit var saturdayDecorator: SaturdayDecorator
    private lateinit var selectedMonthDecorator: SelectedMonthDecorator

    override fun initView() {
        initBottomSheet()
        initEvent()
    }

    private fun initEvent() {
        binding.btnCalendarSelect.setOnClickListener {
            rentViewModel.setRentDate(selectedStartDate, selectedEndDate)
            dismiss()
        }

        // 시작, 종료 범위가 설정되었을 때 리스너
        binding.cvRentCalendar.setOnRangeSelectedListener { _, dates ->
            binding.cvRentCalendar.removeDecorator(dayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)

            val startEndRange =
                listOf(dates.first(), dates.last()).map { CalendarDay.from(it.date) }
            val middleRange = dates.subList(1, dates.size - 1).map { CalendarDay.from(it.date) }

            dayDecorator.setDateRange(startEndRange)
            middleDayDecorator.setDateRange(middleRange)
            binding.cvRentCalendar.addDecorators(
                dayDecorator,
                middleDayDecorator
            )

            selectedStartDate = dates[0].date.format(DateTimeFormatter.ofPattern("MM.dd (E)"))
            selectedEndDate =
                dates[dates.size - 1].date.format(DateTimeFormatter.ofPattern("MM.dd (E)"))
        }

        // 날짜가 단일 선택되었을 때 리스너
        binding.cvRentCalendar.setOnDateChangedListener { widget, date, selected ->
            binding.cvRentCalendar.removeDecorator(dayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)

            val dateRange = listOf(date.date).map {
                CalendarDay.from(it)
            }
            dayDecorator.setDateRange(dateRange)
            binding.cvRentCalendar.addDecorators(
                dayDecorator
            )

            selectedStartDate = date.date.format(DateTimeFormatter.ofPattern("MM.dd (E)"))
            selectedEndDate = ""

            if (!selected) {
                selectedStartDate = ""
            }

            Timber.d("dateRange: $dateRange")
        }
    }

    private fun initBottomSheet() {
        // 요일을 한글로 보이게 설정 월..일 순서로 배치해서 캘린더에는 일..월 순서로 보이도록 설정
        binding.cvRentCalendar.setWeekDayFormatter(ArrayWeekDayFormatter(resources.getTextArray(R.array.custom_weekdays)));

        dayDecorator = DayDecorator(requireActivity(), listOf())
        middleDayDecorator = MiddleDayDecorator(requireActivity(), listOf())
        todayDecorator = TodayDecorator(requireActivity())
        sundayDecorator = SundayDecorator()
        saturdayDecorator = SaturdayDecorator()
        selectedMonthDecorator =
            SelectedMonthDecorator(CalendarDay.today().month, requireActivity())

        // 캘린더에 Decorator 추가
        binding.cvRentCalendar.addDecorators(
            sundayDecorator,
            saturdayDecorator,
            selectedMonthDecorator,
            todayDecorator,
            middleDayDecorator,
            dayDecorator
        )

        // 좌우 화살표 가운데의 연/월이 보이는 방식 지정
        binding.cvRentCalendar.setTitleFormatter { day ->
            val inputText = day.date
            val calendarHeaderElements = inputText.toString().split("-")
            val calendarHeaderBuilder = StringBuilder()

            calendarHeaderBuilder.append(calendarHeaderElements[0]).append("년 ")
                .append(calendarHeaderElements[1]).append("월")

            calendarHeaderBuilder.toString()
        }

        // 캘린더에 보여지는 Month가 변경된 경우
        binding.cvRentCalendar.setOnMonthChangedListener { widget, date ->
            // 기존에 설정되어 있던 Decorators 초기화
            binding.cvRentCalendar.removeDecorators()
            binding.cvRentCalendar.invalidateDecorators()

            // Decorators 추가
            selectedMonthDecorator = SelectedMonthDecorator(date.month, requireActivity())
            binding.cvRentCalendar.addDecorators(
                sundayDecorator,
                saturdayDecorator,
                selectedMonthDecorator,
                todayDecorator,
                middleDayDecorator,
                dayDecorator
            )
        }
    }
}