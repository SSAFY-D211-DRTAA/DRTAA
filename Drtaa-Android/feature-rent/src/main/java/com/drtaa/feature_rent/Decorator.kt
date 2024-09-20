package com.drtaa.feature_rent

import android.content.Context
import android.graphics.Color
import android.text.style.ForegroundColorSpan
import androidx.core.content.ContextCompat
import com.prolificinteractive.materialcalendarview.CalendarDay
import com.prolificinteractive.materialcalendarview.DayViewDecorator
import com.prolificinteractive.materialcalendarview.DayViewFacade
import org.threeten.bp.DayOfWeek

/* 선택된 날짜의 background를 설정하는 클래스 */
class BlockDecorator(context: Context) : DayViewDecorator {
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.gray_d9d9)
    private var today = CalendarDay.today()

    override fun shouldDecorate(day: CalendarDay?): Boolean {
        return day?.isBefore(today)!!
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setDaysDisabled(true)
        view.addSpan(object : ForegroundColorSpan(color) {})
    }
}

/* 선택된 날짜의 background를 설정하는 클래스 */
class SingleDayDecorator(context: Context, private var date: CalendarDay? = null) :
    DayViewDecorator {
    private val drawableSingle =
        ContextCompat.getDrawable(context, R.drawable.selector_calendar_single)
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.white)

    // true를 리턴 시 모든 요일에 내가 설정한 드로어블이 적용된다
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return date == day
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setSelectionDrawable(drawableSingle!!)
        view.addSpan(object : ForegroundColorSpan(color) {})
    }

    fun setDate(date: CalendarDay?) {
        this.date = date
    }
}

/* 선택된 날짜의 background를 설정하는 클래스 */
class StartDayDecorator(context: Context, private var date: CalendarDay? = null) :
    DayViewDecorator {
    private val drawableStart =
        ContextCompat.getDrawable(context, R.drawable.selector_calendar_start)
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.white)

    // true를 리턴 시 모든 요일에 내가 설정한 드로어블이 적용된다
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return date == day
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setSelectionDrawable(drawableStart!!)
        view.addSpan(object : ForegroundColorSpan(color) {})
    }

    fun setDate(date: CalendarDay?) {
        this.date = date
    }
}

/* 선택된 날짜의 background를 설정하는 클래스 */
class EndDayDecorator(context: Context, private var date: CalendarDay? = null) :
    DayViewDecorator {
    private val drawableEnd =
        ContextCompat.getDrawable(context, R.drawable.selector_calendar_end)
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.white)

    // true를 리턴 시 모든 요일에 내가 설정한 드로어블이 적용된다
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return date == day
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setSelectionDrawable(drawableEnd!!)
        view.addSpan(object : ForegroundColorSpan(color) {})
    }

    fun setDate(date: CalendarDay?) {
        this.date = date
    }
}

/* 선택된 날짜 사이의 background를 설정하는 클래스 */
class MiddleDayDecorator(context: Context, private var dateRange: List<CalendarDay>) :
    DayViewDecorator {
    private val drawable = ContextCompat.getDrawable(context, R.drawable.shape_calendar_middle)
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.white)

    // true를 리턴 시 모든 요일에 내가 설정한 드로어블이 적용된다
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return dateRange.contains(day)
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setSelectionDrawable(drawable!!)
        view.addSpan(object : ForegroundColorSpan(color) {})
    }

    fun setDateRange(dateRange: List<CalendarDay>) {
        this.dateRange = dateRange
    }
}

/* 오늘 날짜의 background를 설정하는 클래스 */
class TodayDecorator(context: Context) : DayViewDecorator {
    private val drawable = ContextCompat.getDrawable(context, R.drawable.calendar_circle_blue)
    private val color = ContextCompat.getColor(context, com.drtaa.core_ui.R.color.blue_a0ba)
    private var date = CalendarDay.today()

    override fun shouldDecorate(day: CalendarDay?): Boolean {
        return day?.equals(date)!!
    }

    override fun decorate(view: DayViewFacade?) {
        view?.setBackgroundDrawable(drawable!!)
        view?.addSpan(object : ForegroundColorSpan(color) {})
    }
}

/* 이번달에 속하지 않지만 캘린더에 보여지는 이전달/다음달의 일부 날짜를 설정하는 클래스 */
class SelectedMonthDecorator(private val selectedMonth: Int, val context: Context) :
    DayViewDecorator {
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return day.month != selectedMonth
    }

    override fun decorate(view: DayViewFacade) {
        view.addSpan(
            ForegroundColorSpan(
                ContextCompat.getColor(context, com.drtaa.core_ui.R.color.gray_a3a3)
            )
        )
    }
}

/* 일요일 날짜의 색상을 설정하는 클래스 */
class SundayDecorator : DayViewDecorator {
    private val today = CalendarDay.today()

    override fun shouldDecorate(day: CalendarDay): Boolean {
        val sunday = day.date.with(DayOfWeek.SUNDAY).dayOfMonth
        return sunday == day.day && !day.isBefore(today)
    }

    override fun decorate(view: DayViewFacade) {

        view.addSpan(object : ForegroundColorSpan(Color.RED) {})
    }
}

/* 토요일 날짜의 색상을 설정하는 클래스 */
class SaturdayDecorator : DayViewDecorator {
    private val today = CalendarDay.today()

    override fun shouldDecorate(day: CalendarDay): Boolean {
        val saturday = day.date.with(DayOfWeek.SATURDAY).dayOfMonth
        return saturday == day.day && !day.isBefore(today)
    }

    override fun decorate(view: DayViewFacade) {
        view.addSpan(object : ForegroundColorSpan(Color.BLUE) {})
    }
}