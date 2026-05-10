from __future__ import annotations

import queue
import shutil
import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:  # pragma: no cover - depends on ROS environment
    PackageNotFoundError = RuntimeError
    get_package_share_directory = None

try:
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import GLib, Gst
except (ImportError, ValueError):  # pragma: no cover - depends on host runtime
    GLib = None
    Gst = None


_UNKNOWN_ACTOR_ID = '__unknown_human__'


def default_sound_asset_path(filename: str) -> Path:
    candidate_paths: list[Path] = []
    if get_package_share_directory is not None:
        try:
            candidate_paths.append(Path(get_package_share_directory('hmi_feedback')) / 'sound' / filename)
        except PackageNotFoundError:
            pass
    candidate_paths.append(Path(__file__).resolve().parents[3] / 'sound' / filename)

    for candidate_path in candidate_paths:
        if candidate_path.is_file():
            return candidate_path
    return candidate_paths[0]


def normalize_actor_id(actor_id: str) -> str:
    normalized = actor_id.strip()
    return normalized or _UNKNOWN_ACTOR_ID


@dataclass(frozen=True)
class SoundCueDecision:
    play_soft_chime: bool
    play_hard_stop_warning: bool
    soft_chime_actor_id: str | None = None


class SoundCueTracker:
    def __init__(self) -> None:
        self._last_internal_state = ''
        self._soft_chime_actor_ids: set[str] = set()

    def evaluate(
        self,
        *,
        internal_state: str,
        avoidance_started_event: bool,
        human_actor_id: str,
    ) -> SoundCueDecision:
        entered_conflict_avoiding_navigate = (
            internal_state == 'ConflictAvoidingNavigate'
            and self._last_internal_state != 'ConflictAvoidingNavigate'
        )
        play_hard_stop_warning = internal_state == 'HardStop' and self._last_internal_state != 'HardStop'

        soft_chime_actor_id: str | None = None
        if avoidance_started_event or entered_conflict_avoiding_navigate:
            actor_key = normalize_actor_id(human_actor_id)
            if actor_key not in self._soft_chime_actor_ids:
                self._soft_chime_actor_ids.add(actor_key)
                soft_chime_actor_id = actor_key

        self._last_internal_state = internal_state
        return SoundCueDecision(
            play_soft_chime=soft_chime_actor_id is not None,
            play_hard_stop_warning=play_hard_stop_warning,
            soft_chime_actor_id=soft_chime_actor_id,
        )


class _CueBackend(Protocol):
    name: str

    def play(self, sound_path: Path, stop_event: threading.Event) -> None:
        ...


class _GStreamerCueBackend:
    name = 'gstreamer'

    def __init__(self) -> None:
        if Gst is None or GLib is None:
            raise RuntimeError('GStreamer bindings are unavailable.')
        Gst.init(None)

    def play(self, sound_path: Path, stop_event: threading.Event) -> None:
        if Gst is None or GLib is None:
            raise RuntimeError('GStreamer bindings are unavailable.')

        playbin = Gst.ElementFactory.make('playbin', None)
        if playbin is None:
            raise RuntimeError('Could not create GStreamer playbin.')

        playbin.set_property('uri', GLib.filename_to_uri(str(sound_path), None))
        bus = playbin.get_bus()
        if bus is None:
            raise RuntimeError('Could not create GStreamer bus.')

        try:
            playbin.set_state(Gst.State.PLAYING)
            while not stop_event.is_set():
                message = bus.timed_pop_filtered(
                    100 * Gst.MSECOND,
                    Gst.MessageType.ERROR | Gst.MessageType.EOS,
                )
                if message is None:
                    continue
                if message.type == Gst.MessageType.EOS:
                    return
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = f'{error}'
                    if debug:
                        detail = f'{detail} ({debug})'
                    raise RuntimeError(detail)
        finally:
            playbin.set_state(Gst.State.NULL)


class _FFplayCueBackend:
    name = 'ffplay'

    def __init__(self) -> None:
        executable = shutil.which('ffplay')
        if executable is None:
            raise RuntimeError('ffplay is unavailable.')
        self._executable = executable

    def play(self, sound_path: Path, stop_event: threading.Event) -> None:
        process = subprocess.Popen(
            [
                self._executable,
                '-nodisp',
                '-autoexit',
                '-loglevel',
                'error',
                str(sound_path),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            while process.poll() is None:
                if stop_event.wait(0.05):
                    process.terminate()
                    try:
                        process.wait(timeout=0.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=0.5)
                    return
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=0.5)


@dataclass(order=True)
class _QueuedCue:
    priority: int
    sequence: int
    cue_name: str = field(compare=False)
    sound_path: Path = field(compare=False)


class AudioCuePlayer:
    def __init__(
        self,
        *,
        logger,
        soft_sound_path: Path,
        warning_sound_path: Path,
        enabled: bool = True,
    ) -> None:
        self._logger = logger
        self._soft_sound_path = soft_sound_path
        self._warning_sound_path = warning_sound_path
        self._enabled = enabled
        self._queue: queue.PriorityQueue[_QueuedCue | None] = queue.PriorityQueue()
        self._lock = threading.Lock()
        self._current_stop_event: threading.Event | None = None
        self._sequence = 0
        self._closed = False
        self._backends = self._build_backends()
        self._worker: threading.Thread | None = None

        if not self._enabled:
            self._logger.info('Audio cues disabled by parameter.')
            return
        if not self._backends:
            self._logger.warning('No audio backend available; sound cues will be skipped.')
            return

        self._worker = threading.Thread(target=self._run, name='audio-cue-player', daemon=True)
        self._worker.start()
        backend_names = ', '.join(backend.name for backend in self._backends)
        self._logger.info(f'Audio cue player ready with backends: {backend_names}')

    def enqueue_soft_chime(self) -> None:
        self._enqueue('soft_chime', self._soft_sound_path, priority=1, preempt=False)

    def enqueue_hard_stop_warning(self) -> None:
        self._enqueue('hard_stop_warning', self._warning_sound_path, priority=0, preempt=True)

    def close(self) -> None:
        worker = self._worker
        if worker is None:
            return

        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._current_stop_event is not None:
                self._current_stop_event.set()
        self._queue.put(None)
        worker.join(timeout=1.0)

    def _build_backends(self) -> list[_CueBackend]:
        backends: list[_CueBackend] = []
        for backend_type in (_GStreamerCueBackend, _FFplayCueBackend):
            try:
                backends.append(backend_type())
            except RuntimeError:
                continue
        return backends

    def _enqueue(self, cue_name: str, sound_path: Path, *, priority: int, preempt: bool) -> None:
        if not self._enabled:
            return
        if self._worker is None:
            return
        if not sound_path.is_file():
            self._logger.warning(f'Audio cue file not found for {cue_name}: {sound_path}')
            return

        with self._lock:
            if self._closed:
                return
            self._sequence += 1
            if preempt and self._current_stop_event is not None:
                self._current_stop_event.set()
            queued_cue = _QueuedCue(
                priority=priority,
                sequence=self._sequence,
                cue_name=cue_name,
                sound_path=sound_path,
            )
        self._queue.put(queued_cue)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return

            stop_event = threading.Event()
            with self._lock:
                self._current_stop_event = stop_event
            try:
                self._play_with_fallbacks(item.sound_path, stop_event)
            finally:
                with self._lock:
                    if self._current_stop_event is stop_event:
                        self._current_stop_event = None

    def _play_with_fallbacks(self, sound_path: Path, stop_event: threading.Event) -> None:
        for backend in list(self._backends):
            try:
                backend.play(sound_path, stop_event)
                return
            except RuntimeError as exc:
                if stop_event.is_set():
                    return
                self._logger.warning(f'Audio backend {backend.name} failed: {exc}')
                self._backends = [candidate for candidate in self._backends if candidate.name != backend.name]

        if not stop_event.is_set():
            self._logger.warning(f'Failed to play audio cue: {sound_path}')
