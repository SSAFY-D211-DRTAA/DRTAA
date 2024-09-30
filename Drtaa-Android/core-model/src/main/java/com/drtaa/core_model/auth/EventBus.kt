package com.drtaa.core_model.auth

import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow

class EventBus {
    private val _events = MutableSharedFlow<Event>()
    val events: SharedFlow<Event> = _events

    suspend fun emitEvent(event: Event) {
        _events.emit(event)
    }
}

sealed class Event {
    object LogoutEvent : Event()
}