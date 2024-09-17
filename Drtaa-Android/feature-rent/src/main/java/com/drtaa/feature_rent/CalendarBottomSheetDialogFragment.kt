package com.drtaa.feature_rent

import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_model.data.RentSchedule
import com.drtaa.core_ui.base.BaseBottomSheetDialogFragment
import com.drtaa.feature_rent.databinding.FragmentCalendarBottomSheetBinding
import com.drtaa.feature_rent.util.formatToYearMonthDay
import com.drtaa.feature_rent.viewmodel.CalendarBottomSheetViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import com.prolificinteractive.materialcalendarview.CalendarDay
import com.prolificinteractive.materialcalendarview.format.ArrayWeekDayFormatter
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.threeten.bp.format.DateTimeFormatter

class CalendarBottomSheetDialogFragment :
    BaseBottomSheetDialogFragment<FragmentCalendarBottomSheetBinding>(R.layout.fragment_calendar_bottom_sheet) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val calendarBottomSheetViewModel: CalendarBottomSheetViewModel by viewModels()

    private lateinit var singleDayDecorator: SingleDayDecorator
    private lateinit var startDayDecorator: StartDayDecorator
    private lateinit var endDayDecorator: EndDayDecorator
    private lateinit var middleDayDecorator: MiddleDayDecorator
    private lateinit var todayDecorator: TodayDecorator
    private lateinit var sundayDecorator: SundayDecorator
    private lateinit var saturdayDecorator: SaturdayDecorator
    private lateinit var selectedMonthDecorator: SelectedMonthDecorator

    private lateinit var startTimePickerDialog: TimePickerDialog
    private lateinit var endTimePickerDialog: TimePickerDialog

    override fun initView() {
        initCalendar()
        initCalendarEvent()
        initTimePicker()
        initEvent()
        initObserve()
        initData()
    }

    private fun initData() {
        if (rentViewModel.rentStartSchedule.value != null && rentViewModel.rentEndSchedule.value != null) {
            val startDate = CalendarDay.from(
                rentViewModel.rentStartSchedule.value!!.year,
                rentViewModel.rentStartSchedule.value!!.month,
                rentViewModel.rentStartSchedule.value!!.date
            )
            val endDate = CalendarDay.from(
                rentViewModel.rentEndSchedule.value!!.year,
                rentViewModel.rentEndSchedule.value!!.month,
                rentViewModel.rentEndSchedule.value!!.date
            )

            val startHour = rentViewModel.rentStartSchedule.value!!.hour
            val startMinute = rentViewModel.rentStartSchedule.value!!.minute
            val endHour = rentViewModel.rentEndSchedule.value!!.hour
            val endMinute = rentViewModel.rentEndSchedule.value!!.minute

            binding.cvRentCalendar.selectRange(startDate, endDate)

            calendarBottomSheetViewModel.setRentStartDate(startDate)
            calendarBottomSheetViewModel.setRentStartTime(
                startHour,
                startMinute
            )
            startTimePickerDialog.setTime(
                startHour,
                startMinute
            )

            calendarBottomSheetViewModel.setRentEndDate(endDate)
            calendarBottomSheetViewModel.setRentEndTime(
                endHour,
                endMinute
            )
            endTimePickerDialog.setTime(
                endHour,
                endMinute
            )
        }
    }

    private fun initTimePicker() {
        startTimePickerDialog = TimePickerDialog(requireActivity())
        endTimePickerDialog = TimePickerDialog(requireActivity())

        startTimePickerDialog.onCheckClickListener =
            object : TimePickerDialog.OnCheckClickListener {
                override fun onCheckClick(hour: Int, minute: Int) {
                    startTimePickerDialog.setTime(hour, minute)
                    calendarBottomSheetViewModel.setRentStartTime(hour, minute)
                }
            }

        endTimePickerDialog.onCheckClickListener =
            object : TimePickerDialog.OnCheckClickListener {
                override fun onCheckClick(hour: Int, minute: Int) {
                    endTimePickerDialog.setTime(hour, minute)
                    calendarBottomSheetViewModel.setRentEndTime(hour, minute)
                }
            }
    }

    private fun initObserve() {
        calendarBottomSheetViewModel.rentStartTime.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentStartTime ->
                binding.tvRentSummaryStartTime.text = rentStartTime?.toString() ?: ""
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        calendarBottomSheetViewModel.rentEndTime.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { rentEndTime ->
                binding.tvRentSummaryEndTime.text = rentEndTime?.toString()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        calendarBottomSheetViewModel.rentStartDate.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { startDate ->
                binding.tvRentSummaryStartDate.text = startDate.formatToYearMonthDay()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        calendarBottomSheetViewModel.rentEndDate.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { endDate ->
                binding.tvRentSummaryEndDate.text = endDate.formatToYearMonthDay()
            }.launchIn(viewLifecycleOwner.lifecycleScope)

        calendarBottomSheetViewModel.isScheduleValid.flowWithLifecycle(viewLifecycleOwner.lifecycle)
            .onEach { isValid ->
                binding.btnCalendarSelect.isEnabled = isValid
            }.launchIn(viewLifecycleOwner.lifecycleScope)
    }

    private fun initEvent() {
        binding.btnCalendarSelect.setOnClickListener {
            rentViewModel.setRentStartSchedule(
                RentSchedule(
                    year = calendarBottomSheetViewModel.rentStartDate.value!!.year,
                    month = calendarBottomSheetViewModel.rentStartDate.value!!.month,
                    date = calendarBottomSheetViewModel.rentStartDate.value!!.day,
                    day = calendarBottomSheetViewModel.rentStartDate.value!!.date.format(
                        DateTimeFormatter.ofPattern("E")
                    ),
                    hour = calendarBottomSheetViewModel.rentStartTime.value!!.hour,
                    minute = calendarBottomSheetViewModel.rentStartTime.value!!.minute
                )
            )
            rentViewModel.setRentEndSchedule(
                RentSchedule(
                    year = calendarBottomSheetViewModel.rentEndDate.value!!.year,
                    month = calendarBottomSheetViewModel.rentEndDate.value!!.month,
                    date = calendarBottomSheetViewModel.rentEndDate.value!!.day,
                    day = calendarBottomSheetViewModel.rentEndDate.value!!.date.format(
                        DateTimeFormatter.ofPattern("E")
                    ),
                    hour = calendarBottomSheetViewModel.rentEndTime.value!!.hour,
                    minute = calendarBottomSheetViewModel.rentEndTime.value!!.minute
                )
            )

            dismiss()
        }

        binding.tvRentSummaryStartTime.setOnClickListener {
            startTimePickerDialog.show()
        }

        binding.tvRentSummaryEndTime.setOnClickListener {
            endTimePickerDialog.show()
        }
    }

    private fun initCalendar() {
        // 요일을 한글로 보이게 설정 월..일 순서로 배치해서 캘린더에는 일..월 순서로 보이도록 설정
        binding.cvRentCalendar.setWeekDayFormatter(
            ArrayWeekDayFormatter(resources.getTextArray(R.array.custom_weekdays))
        )

        singleDayDecorator = SingleDayDecorator(requireActivity())
        startDayDecorator = StartDayDecorator(requireActivity())
        endDayDecorator = EndDayDecorator(requireActivity())
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
            startDayDecorator,
            endDayDecorator,
            singleDayDecorator
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
    }

    private fun initCalendarEvent() {
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
                startDayDecorator,
                endDayDecorator,
                singleDayDecorator
            )
        }

        // 시작, 종료 범위가 설정되었을 때 리스너
        binding.cvRentCalendar.setOnRangeSelectedListener { _, dates ->
            binding.cvRentCalendar.removeDecorator(singleDayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)
            binding.cvRentCalendar.removeDecorator(startDayDecorator)
            binding.cvRentCalendar.removeDecorator(endDayDecorator)

            val middleRange = dates.subList(1, dates.size - 1).map { CalendarDay.from(it.date) }

            startDayDecorator.setDate(dates.first())
            endDayDecorator.setDate(dates.last())
            middleDayDecorator.setDateRange(middleRange)
            binding.cvRentCalendar.addDecorators(
                startDayDecorator,
                endDayDecorator,
                middleDayDecorator
            )

            calendarBottomSheetViewModel.setRentStartDate(dates[0])
            calendarBottomSheetViewModel.setRentEndDate(dates[dates.size - 1])
        }

        // 날짜가 단일 선택되었을 때 리스너
        binding.cvRentCalendar.setOnDateChangedListener { widget, date, selected ->
            binding.cvRentCalendar.removeDecorator(singleDayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)
            binding.cvRentCalendar.removeDecorator(startDayDecorator)
            binding.cvRentCalendar.removeDecorator(endDayDecorator)

            singleDayDecorator.setDate(date)
            binding.cvRentCalendar.addDecorators(
                singleDayDecorator
            )

            calendarBottomSheetViewModel.setRentStartDate(date)
            calendarBottomSheetViewModel.setRentEndDate(null)

            if (!selected) {
                calendarBottomSheetViewModel.setRentStartDate(null)
                binding.cvRentCalendar.removeDecorator(singleDayDecorator)
            }
        }
    }
}