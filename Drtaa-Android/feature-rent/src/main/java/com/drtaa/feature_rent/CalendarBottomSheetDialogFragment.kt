package com.drtaa.feature_rent

import android.app.Dialog
import android.os.Bundle
import android.view.View
import android.widget.FrameLayout
import androidx.fragment.app.viewModels
import androidx.hilt.navigation.fragment.hiltNavGraphViewModels
import androidx.lifecycle.flowWithLifecycle
import androidx.lifecycle.lifecycleScope
import com.drtaa.core_model.rent.RentSchedule
import com.drtaa.core_ui.base.BaseBottomSheetDialogFragment
import com.drtaa.feature_rent.databinding.FragmentCalendarBottomSheetBinding
import com.drtaa.feature_rent.util.formatToYearMonthDay
import com.drtaa.feature_rent.viewmodel.CalendarBottomSheetViewModel
import com.drtaa.feature_rent.viewmodel.RentViewModel
import com.google.android.material.bottomsheet.BottomSheetBehavior
import com.google.android.material.bottomsheet.BottomSheetDialog
import com.prolificinteractive.materialcalendarview.CalendarDay
import com.prolificinteractive.materialcalendarview.format.ArrayWeekDayFormatter
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.threeten.bp.format.DateTimeFormatter

class CalendarBottomSheetDialogFragment :
    BaseBottomSheetDialogFragment<FragmentCalendarBottomSheetBinding>(R.layout.fragment_calendar_bottom_sheet) {

    private val rentViewModel: RentViewModel by hiltNavGraphViewModels(R.id.nav_graph_rent)
    private val calendarBottomSheetViewModel: CalendarBottomSheetViewModel by viewModels()

    private lateinit var blockDecorator: BlockDecorator
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

    override fun onCreateDialog(savedInstanceState: Bundle?): Dialog {
        val dialog = super.onCreateDialog(savedInstanceState) as BottomSheetDialog
        dialog.setOnShowListener { dialogInterface ->
            val bottomSheetDialog = dialogInterface as BottomSheetDialog
            setupFullHeight(bottomSheetDialog)
        }
        return dialog
    }

    override fun initView() {
        initCalendar()
        initCalendarEvent()
        initTimePicker()
        initEvent()
        initObserve()
        initData()
    }

    private fun setupFullHeight(bottomSheetDialog: BottomSheetDialog) {
        val bottomSheet =
            bottomSheetDialog.findViewById<View>(com.google.android.material.R.id.design_bottom_sheet) as FrameLayout?
        val behavior = BottomSheetBehavior.from(bottomSheet!!)
        val layoutParams = bottomSheet.layoutParams
        bottomSheet.layoutParams = layoutParams
        behavior.state = BottomSheetBehavior.STATE_EXPANDED
        behavior.skipCollapsed = true
    }

    private fun initData() {
        val rentStartSchedule = rentViewModel.rentStartSchedule.value
        val rentEndSchedule = rentViewModel.rentEndSchedule.value

        if (rentStartSchedule != null && rentEndSchedule != null) {
            val startDate = CalendarDay.from(
                rentStartSchedule.year,
                rentStartSchedule.month,
                rentStartSchedule.date
            )
            val endDate = CalendarDay.from(
                rentEndSchedule.year,
                rentEndSchedule.month,
                rentEndSchedule.date
            )

            val startHour = rentStartSchedule.hour
            val startMinute = rentStartSchedule.minute
            val endHour = rentEndSchedule.hour
            val endMinute = rentEndSchedule.minute

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
            val rentStartDate = calendarBottomSheetViewModel.rentStartDate.value
            val rentStartTime = calendarBottomSheetViewModel.rentStartTime.value
            val rentEndDate = calendarBottomSheetViewModel.rentEndDate.value
            val rentEndTime = calendarBottomSheetViewModel.rentEndTime.value

            if (rentStartDate != null && rentEndDate != null && rentStartTime != null && rentEndTime != null) {
                rentViewModel.setRentStartSchedule(
                    RentSchedule(
                        year = rentStartDate.year,
                        month = rentStartDate.month,
                        date = rentStartDate.day,
                        day = rentStartDate.date.format(
                            DateTimeFormatter.ofPattern("E")
                        ),
                        hour = rentStartTime.hour,
                        minute = rentStartTime.minute
                    )
                )
                rentViewModel.setRentEndSchedule(
                    RentSchedule(
                        year = rentEndDate.year,
                        month = rentEndDate.month,
                        date = rentEndDate.day,
                        day = rentEndDate.date.format(
                            DateTimeFormatter.ofPattern("E")
                        ),
                        hour = rentEndTime.hour,
                        minute = rentEndTime.minute
                    )
                )
                dismiss()
            }
        }

        binding.llRentSummaryStart.setOnClickListener {
            startTimePickerDialog.show()
        }

        binding.llRentSummaryEnd.setOnClickListener {
            endTimePickerDialog.show()
        }
    }

    private fun initCalendar() {
        // 요일을 한글로 보이게 설정 월..일 순서로 배치해서 캘린더에는 일..월 순서로 보이도록 설정
        binding.cvRentCalendar.setWeekDayFormatter(
            ArrayWeekDayFormatter(resources.getTextArray(R.array.custom_weekdays))
        )

        blockDecorator = BlockDecorator(requireActivity())
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
            blockDecorator,
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
                blockDecorator,
                sundayDecorator,
                saturdayDecorator,
                selectedMonthDecorator,
                todayDecorator,
                singleDayDecorator,
                middleDayDecorator,
                startDayDecorator,
                endDayDecorator
            )
        }

        // 시작, 종료 범위가 설정되었을 때 리스너
        binding.cvRentCalendar.setOnRangeSelectedListener { _, dates ->
            binding.cvRentCalendar.removeDecorator(singleDayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)
            binding.cvRentCalendar.removeDecorator(startDayDecorator)
            binding.cvRentCalendar.removeDecorator(endDayDecorator)
            binding.cvRentCalendar.invalidateDecorators()

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
            calendarBottomSheetViewModel.setRentEndDate(null)
            if (selected) {
                singleDayDecorator.setDate(date)
                calendarBottomSheetViewModel.setRentStartDate(date)
            } else {
                singleDayDecorator.setDate(null)
                calendarBottomSheetViewModel.setRentStartDate(null)
            }

            startDayDecorator.setDate(null)
            middleDayDecorator.setDateRange(listOf())
            endDayDecorator.setDate(null)

            binding.cvRentCalendar.removeDecorator(singleDayDecorator)
            binding.cvRentCalendar.removeDecorator(middleDayDecorator)
            binding.cvRentCalendar.removeDecorator(startDayDecorator)
            binding.cvRentCalendar.removeDecorator(endDayDecorator)
            binding.cvRentCalendar.invalidateDecorators()

            if (selected) {
                binding.cvRentCalendar.addDecorators(singleDayDecorator)
            }
        }
    }
}