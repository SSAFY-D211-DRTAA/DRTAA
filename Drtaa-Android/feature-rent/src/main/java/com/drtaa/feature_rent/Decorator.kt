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
class DayDecorator(context: Context) : DayViewDecorator {
    private val drawable = ContextCompat.getDrawable(context, R.drawable.selector_calendar_range)

    // true를 리턴 시 모든 요일에 내가 설정한 드로어블이 적용된다
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return true
    }

    // 일자 선택 시 내가 정의한 드로어블이 적용되도록 한다
    override fun decorate(view: DayViewFacade) {
        view.setSelectionDrawable(drawable!!)
    }
}

/* 오늘 날짜의 background를 설정하는 클래스 */
class TodayDecorator(context: Context) : DayViewDecorator {
    private val drawable = ContextCompat.getDrawable(context, R.drawable.calendar_circle_blue)
    private var date = CalendarDay.today()
    override fun shouldDecorate(day: CalendarDay?): Boolean {
        return day?.equals(date)!!
    }

    override fun decorate(view: DayViewFacade?) {
        view?.setBackgroundDrawable(drawable!!)
    }
}

/* 이번달에 속하지 않지만 캘린더에 보여지는 이전달/다음달의 일부 날짜를 설정하는 클래스 */
class SelectedMonthDecorator(val selectedMonth: Int, val context: Context) : DayViewDecorator {
    override fun shouldDecorate(day: CalendarDay): Boolean {
        return day.month != selectedMonth
    }

    override fun decorate(view: DayViewFacade) {
        view.addSpan(
            ForegroundColorSpan(
                ContextCompat.getColor(context, com.drtaa.core_ui.R.color.gray_d9d9)
            )
        )
    }
}

/* 일요일 날짜의 색상을 설정하는 클래스 */
class SundayDecorator : DayViewDecorator {
    override fun shouldDecorate(day: CalendarDay): Boolean {
        val sunday = day.date.with(DayOfWeek.SUNDAY).dayOfMonth
        return sunday == day.day
    }

    override fun decorate(view: DayViewFacade) {
        view.addSpan(object : ForegroundColorSpan(Color.RED) {})
    }
}

/* 토요일 날짜의 색상을 설정하는 클래스 */
class SaturdayDecorator : DayViewDecorator {
    override fun shouldDecorate(day: CalendarDay): Boolean {
        val saturday = day.date.with(DayOfWeek.SATURDAY).dayOfMonth
        return saturday == day.day
    }

    override fun decorate(view: DayViewFacade) {
        view.addSpan(object : ForegroundColorSpan(Color.BLUE) {})
    }
}