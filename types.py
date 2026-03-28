from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import TYPE_CHECKING, Callable, Protocol

from hackarena3.proto.race.v1 import race_pb2, telemetry_pb2, track_pb2

if TYPE_CHECKING:
    from hackarena3.proto.race.v1.telemetry_pb2 import ParticipantSnapshot


class _OpenIntEnum(IntEnum):
    """Rozszerzony IntEnum, który nie rzuca wyjątku dla nieznanych wartości —
    zamiast tego tworzy dynamiczny element o nazwie UNKNOWN_<wartość>."""

    @classmethod
    def _missing_(cls, value: object) -> _OpenIntEnum:
        if not isinstance(value, int):
            raise ValueError(f"{value!r} is not a valid {cls.__name__}")
        member = int.__new__(cls, value)
        member._name_ = f"UNKNOWN_{value}"
        member._value_ = value
        return member


class GearShift(_OpenIntEnum):
    """Żądana zmiana biegu w danym ticku.

    NONE      – brak zmiany biegu
    UPSHIFT   – wrzuć wyższy bieg
    DOWNSHIFT – wrzuć niższy bieg
    """
    NONE = int(race_pb2.GEAR_SHIFT_NONE)
    UPSHIFT = int(race_pb2.GEAR_SHIFT_UPSHIFT)
    DOWNSHIFT = int(race_pb2.GEAR_SHIFT_DOWNSHIFT)


class DriveGear(_OpenIntEnum):
    """Aktualnie wrzucony bieg.

    REVERSE – wsteczny
    NEUTRAL – luz
    FIRST–EIGHTH – biegi od 1 do 8
    """
    REVERSE = -1
    NEUTRAL = 0
    FIRST = 1
    SECOND = 2
    THIRD = 3
    FOURTH = 4
    FIFTH = 5
    SIXTH = 6
    SEVENTH = 7
    EIGHTH = 8


class TireType(_OpenIntEnum):
    """Typ opon zamontowanych na samochodzie lub zaplanowanych na pit stopie.

    UNSPECIFIED – nieokreślony (wartość domyślna/błędna)
    HARD        – opony twarde (mniejszy zużycie, mniejsza przyczepność)
    SOFT        – opony miękkie (większa przyczepność, szybsze zużycie)
    WET         – opony deszczowe
    """
    UNSPECIFIED = int(telemetry_pb2.TIRE_TYPE_UNSPECIFIED)
    HARD = int(telemetry_pb2.TIRE_TYPE_HARD)
    SOFT = int(telemetry_pb2.TIRE_TYPE_SOFT)
    WET = int(telemetry_pb2.TIRE_TYPE_WET)


class GroundType(_OpenIntEnum):
    """Rodzaj nawierzchni w danym pasie toru.

    ASPHALT – asfalt (główna jezdnia, strefa ucieczkowa)
    GRASS   – trawa
    GRAVEL  – żwir
    WALL    – ściana/bariera
    KERB    – krawężnik
    """
    ASPHALT = int(track_pb2.GROUND_TYPE_ASPHALT)
    GRASS = int(track_pb2.GROUND_TYPE_GRASS)
    GRAVEL = int(track_pb2.GROUND_TYPE_GRAVEL)
    WALL = int(track_pb2.GROUND_TYPE_WALL)
    KERB = int(track_pb2.GROUND_TYPE_KERB)


class GhostModePhase(_OpenIntEnum):
    """Faza trybu-widmo (ghost mode) — mechanizmu zapobiegającego kolizjom.

    UNSPECIFIED   – nieokreślona (wartość domyślna)
    INACTIVE      – tryb nieaktywny, samochód normalnie koliduje
    ACTIVE        – tryb aktywny, samochód przenika przez innych
    PENDING_EXIT  – oczekiwanie na wyjście z trybu (spełnienie warunków)
    """
    UNSPECIFIED = int(telemetry_pb2.GHOST_MODE_PHASE_UNSPECIFIED)
    INACTIVE = int(telemetry_pb2.GHOST_MODE_PHASE_INACTIVE)
    ACTIVE = int(telemetry_pb2.GHOST_MODE_PHASE_ACTIVE)
    PENDING_EXIT = int(telemetry_pb2.GHOST_MODE_PHASE_PENDING_EXIT)


class GhostModeBlocker(_OpenIntEnum):
    """Powód blokujący wyjście z trybu-widmo.

    UNSPECIFIED              – nieokreślony
    LAPS_REQUIREMENT_NOT_MET – nie ukończono wymaganej liczby okrążeń
    EXIT_SPEED_NOT_MET       – prędkość zbyt niska, by bezpiecznie wyjść
    EXIT_DELAY_RUNNING       – trwa odliczanie opóźnienia wyjścia
    VEHICLE_OVERLAP_ACTIVE   – aktualnie zachodzi nakładanie z innym pojazdem
    OVERLAP_EXIT_DELAY_RUNNING – odliczanie po ustaniu nakładania
    IN_PIT                   – samochód znajduje się w alei serwisowej
    """
    UNSPECIFIED = int(telemetry_pb2.GHOST_MODE_BLOCKER_UNSPECIFIED)
    LAPS_REQUIREMENT_NOT_MET = int(telemetry_pb2.GHOST_MODE_BLOCKER_LAPS_REQUIREMENT_NOT_MET)
    EXIT_SPEED_NOT_MET = int(telemetry_pb2.GHOST_MODE_BLOCKER_EXIT_SPEED_NOT_MET)
    EXIT_DELAY_RUNNING = int(telemetry_pb2.GHOST_MODE_BLOCKER_EXIT_DELAY_RUNNING)
    VEHICLE_OVERLAP_ACTIVE = int(telemetry_pb2.GHOST_MODE_BLOCKER_VEHICLE_OVERLAP_ACTIVE)
    OVERLAP_EXIT_DELAY_RUNNING = int(telemetry_pb2.GHOST_MODE_BLOCKER_OVERLAP_EXIT_DELAY_RUNNING)
    IN_PIT = int(telemetry_pb2.GHOST_MODE_BLOCKER_IN_PIT)


class PitstopZoneFlag(IntFlag):
    """Flagi opisujące, w której strefie pit stopu aktualnie znajduje się samochód.
    Wartości można łączyć bitowo (IntFlag).

    NONE / UNSPECIFIED – poza strefą pit stopu
    ENTER              – strefa wjazdu do alei serwisowej
    FIX                – strefa obsługi (wymiana opon, naprawa)
    EXIT               – strefa wyjazdu z alei serwisowej
    """
    NONE = 0
    ENTER = int(telemetry_pb2.PITSTOP_ZONE_FLAG_ENTER)
    FIX = int(telemetry_pb2.PITSTOP_ZONE_FLAG_FIX)
    EXIT = int(telemetry_pb2.PITSTOP_ZONE_FLAG_EXIT)
    UNSPECIFIED = NONE


class PitEntrySource(_OpenIntEnum):
    """Źródło decyzji o wjeździe do pit stopu.

    UNSPECIFIED  – nieokreślone
    BOT_DECISION – bot samodzielnie zdecydował o pit stopie
    REQUESTED    – pit stop zainicjowany przez wywołanie request_*
    EMERGENCY    – awaryjny pit stop (np. krytyczne zużycie opon)
    """
    UNSPECIFIED = int(telemetry_pb2.PIT_ENTRY_SOURCE_UNSPECIFIED)
    BOT_DECISION = int(telemetry_pb2.PIT_ENTRY_SOURCE_BOT_DECISION)
    REQUESTED = int(telemetry_pb2.PIT_ENTRY_SOURCE_REQUESTED)
    EMERGENCY = int(telemetry_pb2.PIT_ENTRY_SOURCE_EMERGENCY)


@dataclass(frozen=True, slots=True)
class Vec3:
    """Wektor lub punkt w przestrzeni 3D.

    x – składowa pozioma (prawo/lewo)
    y – składowa pozioma (przód/tył)
    z – składowa pionowa (góra/dół)
    """
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class CarDimensions:
    """Wymiary fizyczne samochodu.

    width_m – szerokość pojazdu w metrach
    depth_m – długość (głębokość) pojazdu w metrach
    """
    width_m: float
    depth_m: float


@dataclass(frozen=True, slots=True)
class GroundWidth:
    """Pas nawierzchni o jednorodnym typie, prostopadle do osi toru.

    width_m     – szerokość pasa w metrach
    ground_type – rodzaj nawierzchni (GroundType)
    """
    width_m: float
    ground_type: GroundType


@dataclass(frozen=True, slots=True)
class CenterlinePoint:
    """Jeden punkt na osi centralnej toru wraz z pełnym opisem geometrii.

    s_m             – dystans wzdłuż osi toru od linii startowej [m]
    position        – pozycja punktu w układzie świata [Vec3, metry]
    tangent         – jednostkowy wektor styczny (kierunek jazdy)
    normal          – jednostkowy wektor normalny (w górę od nawierzchni)
    right           – jednostkowy wektor w prawo (prostopadle do jazdy)
    left_width_m    – użyteczna szerokość toru po lewej stronie osi [m]
    right_width_m   – użyteczna szerokość toru po prawej stronie osi [m]
    curvature_1pm   – krzywizna toru [1/m]; >0 = zakręt w lewo, <0 = prawo
    grade_rad       – nachylenie wzdłużne (wzniesienie/spadek) [rad]
    bank_rad        – przechyłka poprzeczna nawierzchni [rad]
    max_left_width_m  – maksymalna szerokość po lewej łącznie z poboczem [m]
    max_right_width_m – maksymalna szerokość po prawej łącznie z poboczem [m]
    left_grounds    – kolejne pasy nawierzchni na lewo od osi (od środka)
    right_grounds   – kolejne pasy nawierzchni na prawo od osi (od środka)
    """
    s_m: float
    position: Vec3
    tangent: Vec3
    normal: Vec3
    right: Vec3
    left_width_m: float
    right_width_m: float
    curvature_1pm: float
    grade_rad: float
    bank_rad: float
    max_left_width_m: float
    max_right_width_m: float
    left_grounds: tuple[GroundWidth, ...]
    right_grounds: tuple[GroundWidth, ...]


@dataclass(frozen=True, slots=True)
class PitstopLayout:
    """Geometria alei serwisowej podzielona na trzy strefy.

    enter    – punkty osi strefy wjazdu do pit lanu
    fix      – punkty osi strefy obsługi (box)
    exit     – punkty osi strefy wyjazdu z pit lanu
    length_m – całkowita długość alei serwisowej [m]
    """
    enter: tuple[CenterlinePoint, ...]
    fix: tuple[CenterlinePoint, ...]
    exit: tuple[CenterlinePoint, ...]
    length_m: float


@dataclass(frozen=True, slots=True)
class TrackLayout:
    """Pełny opis geometrii toru wyścigowego.

    map_id       – unikalny identyfikator mapy/toru
    lap_length_m – długość jednego okrążenia [m]
    centerline   – ciąg punktów osi centralnej toru (patrz CenterlinePoint)
    pitstop      – geometria alei serwisowej (patrz PitstopLayout)
    """
    map_id: str
    lap_length_m: float
    centerline: tuple[CenterlinePoint, ...]
    pitstop: PitstopLayout


@dataclass(frozen=True, slots=True)
class Controls:
    """Sterowanie samochodem wysyłane do silnika w danym ticku.

    throttle         – gaz [0.0–1.0]; 0 = brak gazu, 1 = pełny gaz
    brake            – hamulec [0.0–1.0]; 0 = brak hamowania, 1 = pełne hamowanie
    steering         – skręt kierownicy [-1.0–1.0]; -1 = pełny lewo, 1 = pełny prawo
    gear_shift       – żądana zmiana biegu w tym ticku (domyślnie NONE)
    brake_balancer   – rozkład siły hamowania przód/tył [0.0–1.0];
                       0 = tylko tył, 1 = tylko przód, 0.5 = równo (domyślnie)
    differential_lock – stopień blokady dyferencjału [0.0–1.0];
                       0 = otwarty dyferencjał, 1 = pełna blokada (domyślnie 0)
    """
    throttle: float
    brake: float
    steering: float
    gear_shift: GearShift = GearShift.NONE
    brake_balancer: float = 0.5
    differential_lock: float = 0.0


@dataclass(frozen=True, slots=True)
class Quaternion:
    """Kwaternion reprezentujący orientację obiektu w przestrzeni 3D.

    x, y, z – składowe wektorowe kwaterniona
    w        – składowa skalarna kwaterniona
    """
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class GhostModeState:
    """Stan trybu-widmo pojazdu.

    can_collide_now          – True jeśli pojazd może aktualnie kolidować z innymi
    phase                    – aktualna faza trybu-widmo (GhostModePhase)
    blockers                 – zestaw aktywnych blokad wyjścia z trybu-widmo
    exit_delay_remaining_ms  – pozostały czas opóźnienia wyjścia z trybu [ms]

    Właściwość:
    is_ghost – True jeśli pojazd aktualnie przenika przez innych (skrót dla not can_collide_now)
    """
    can_collide_now: bool
    phase: GhostModePhase
    blockers: tuple[GhostModeBlocker, ...]
    exit_delay_remaining_ms: int

    @property
    def is_ghost(self) -> bool:
        """True gdy pojazd jest w trybie-widmo (nie może kolidować)."""
        return not self.can_collide_now


@dataclass(frozen=True, slots=True)
class TireWearPerWheel:
    """Zużycie opon osobno dla każdego koła [0.0–1.0]; 1.0 = opona nowa, 0.0 = całkowicie zużyta.

    front_left  – lewe przednie koło
    front_right – prawe przednie koło
    rear_left   – lewe tylne koło
    rear_right  – prawe tylne koło
    """
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float


@dataclass(frozen=True, slots=True)
class TireTemperaturePerWheel:
    """Temperatura opon osobno dla każdego koła [°C].
    Opony działają najlepiej w optymalnym zakresie temperatur.

    front_left_celsius  – lewe przednie koło
    front_right_celsius – prawe przednie koło
    rear_left_celsius   – lewe tylne koło
    rear_right_celsius  – prawe tylne koło
    """
    front_left_celsius: float
    front_right_celsius: float
    rear_left_celsius: float
    rear_right_celsius: float


@dataclass(frozen=True, slots=True)
class TireSlipPerWheel:
    """Poślizg opon osobno dla każdego koła [0.0+]; 0 = brak poślizgu.
    Wysokie wartości oznaczają utratę przyczepności (blokada lub spin).

    front_left  – lewe przednie koło
    front_right – prawe przednie koło
    rear_left   – lewe tylne koło
    rear_right  – prawe tylne koło
    """
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float


@dataclass(frozen=True, slots=True)
class CommandCooldownState:
    """Pozostałe czasy cooldownu dla komend specjalnych.

    back_to_track_remaining_ms      – czas do ponownego użycia komendy powrotu na tor [ms]
    emergency_pitstop_remaining_ms  – czas do ponownego użycia awaryjnego pit stopu [ms]
    """
    back_to_track_remaining_ms: int
    emergency_pitstop_remaining_ms: int


@dataclass(frozen=True, slots=True)
class CarState:
    """Pełny stan pojazdu gracza w danym ticku.

    car_id                       – unikalny identyfikator pojazdu
    position                     – pozycja w układzie świata [Vec3, metry]
    orientation                  – orientacja pojazdu jako kwaternion
    speed_mps                    – prędkość chwilowa [m/s]
    gear                         – aktualnie wrzucony bieg (DriveGear)
    engine_rpm                   – obroty silnika [RPM]
    last_applied_client_seq      – numer sekwencyjny ostatnio zastosowanych Controls
    pitstop_zone_flags           – flagi strefy pit stopu (PitstopZoneFlag)
    wheels_in_pitstop            – liczba kół aktualnie w strefie pit stopu [0–4]
    ghost_mode                   – stan trybu-widmo (GhostModeState)
    tire_type                    – aktualnie zamontowany typ opon
    next_pit_tire_type           – typ opon zaplanowanych na następny pit stop
    tire_wear                    – zużycie opon per koło (TireWearPerWheel)
    tire_temperature_celsius     – temperatura opon per koło (TireTemperaturePerWheel)
    tire_slip                    – poślizg opon per koło (TireSlipPerWheel)
    pit_request_active           – True jeśli aktywne jest żądanie wjazdu do pit stopu
    pit_emergency_lock_remaining_ms – czas blokady po awaryjnym pit stopie [ms]
    last_pit_time_ms             – czas serwera ostatniego pit stopu [ms]
    last_pit_source              – źródło ostatniej decyzji o pit stopie (PitEntrySource)
    last_pit_lap                 – numer okrążenia ostatniego pit stopu
    command_cooldowns            – stany cooldownów komend specjalnych

    Właściwość:
    speed_kmh – prędkość chwilowa [km/h] (przelicznik ze speed_mps)
    """
    car_id: int
    position: Vec3
    orientation: Quaternion
    speed_mps: float
    gear: DriveGear
    engine_rpm: float
    last_applied_client_seq: int
    pitstop_zone_flags: PitstopZoneFlag
    wheels_in_pitstop: int
    ghost_mode: GhostModeState
    tire_type: TireType
    next_pit_tire_type: TireType
    tire_wear: TireWearPerWheel
    tire_temperature_celsius: TireTemperaturePerWheel
    tire_slip: TireSlipPerWheel
    pit_request_active: bool
    pit_emergency_lock_remaining_ms: int
    last_pit_time_ms: int
    last_pit_source: PitEntrySource
    last_pit_lap: int
    command_cooldowns: CommandCooldownState

    @property
    def speed_kmh(self) -> float:
        """Prędkość chwilowa przeliczona na km/h."""
        return self.speed_mps * 3.6


@dataclass(frozen=True, slots=True)
class OpponentState:
    """Uproszczony stan pojazdu przeciwnika (widoczny dla bota).

    car_id      – unikalny identyfikator pojazdu przeciwnika
    position    – pozycja w układzie świata [Vec3, metry]
    orientation – orientacja pojazdu jako kwaternion
    ghost_mode  – stan trybu-widmo przeciwnika (GhostModeState)
    """
    car_id: int
    position: Vec3
    orientation: Quaternion
    ghost_mode: GhostModeState


@dataclass(frozen=True, slots=True)
class RaceSnapshot:
    """Migawka stanu wyścigu dostarczona botowi w danym ticku.

    tick           – numer ticku (rośnie monotonicznie od startu wyścigu)
    server_time_ms – czas serwera w chwili wygenerowania migawki [ms]
    car            – pełny stan pojazdu gracza (CarState)
    opponents      – krotka ze stanami pojazdów przeciwników (OpponentState)
    raw            – surowy protobuf ParticipantSnapshot (do zaawansowanego użytku)
    """
    tick: int
    server_time_ms: int
    car: CarState
    opponents: tuple[OpponentState, ...]
    raw: ParticipantSnapshot


@dataclass(slots=True)
class RuntimeConfig:
    """Konfiguracja połączenia z serwerem wyścigowym.

    api_addr    – adres serwera API (host:port)
    ha_auth_bin – opcjonalny binarny token autoryzacyjny HackArena
    sandbox_id  – opcjonalny identyfikator piaskownicy (sandbox) do izolacji sesji
    """
    api_addr: str
    ha_auth_bin: str | None = None
    sandbox_id: str | None = None


def _unbound_set_controls(controls: Controls) -> None:
    _ = controls
    raise RuntimeError("BotContext is not attached to active runtime.")


def _unbound_command() -> None:
    raise RuntimeError("BotContext is not attached to active runtime.")


def _unbound_set_next_pit_tire_type(tire_type: TireType) -> None:
    _ = tire_type
    raise RuntimeError("BotContext is not attached to active runtime.")


@dataclass(frozen=True, slots=True)
class _BotContextActions:
    """Wewnętrzne callbacki akcji bota, wstrzykiwane przez środowisko wykonawcze.
    Domyślne implementacje rzucają RuntimeError (zabezpieczenie przed użyciem
    przed podłączeniem do aktywnego runtime'u).

    set_controls            – wyślij Controls do silnika
    request_back_to_track   – zażądaj powrotu na tor po wypadku
    request_emergency_pitstop – zażądaj awaryjnego pit stopu
    set_next_pit_tire_type  – ustaw typ opon na następny pit stop
    """
    set_controls: Callable[[Controls], None] = _unbound_set_controls
    request_back_to_track: Callable[[], None] = _unbound_command
    request_emergency_pitstop: Callable[[], None] = _unbound_command
    set_next_pit_tire_type: Callable[[TireType], None] = _unbound_set_next_pit_tire_type


@dataclass(slots=True)
class BotContext:
    """Kontekst bota dostępny przez całą sesję wyścigową.
    Przekazywany do on_tick() razem z RaceSnapshot.

    car_id         – identyfikator pojazdu przypisanego do bota
    map_id         – identyfikator aktualnej mapy/toru
    car_dimensions – wymiary fizyczne pojazdu (szerokość, długość)
    requested_hz   – żądana częstotliwość ticków [Hz]
    track          – pełny opis geometrii toru (TrackLayout)
    effective_hz   – rzeczywista częstotliwość ticków [Hz] lub None przed ustaleniem
    tick           – numer aktualnego ticku
    _actions       – wewnętrzne callbacki (nie używać bezpośrednio)

    Metody:
    set_controls(...)          – wyślij sterowanie do silnika w tym ticku
    request_back_to_track()    – wróć na tor po wypadnięciu
    request_emergency_pitstop() – natychmiastowy wjazd do pit stopu
    set_next_pit_tire_type(t)  – zmień typ opon na następny pit stop
    """
    car_id: int
    map_id: str
    car_dimensions: CarDimensions
    requested_hz: int
    track: TrackLayout
    effective_hz: int | None
    tick: int
    _actions: _BotContextActions = field(
        default_factory=_BotContextActions,
        repr=False,
    )

    def set_controls(
        self,
        *,
        throttle: float,
        brake: float,
        steer: float,
        gear_shift: GearShift = GearShift.NONE,
        brake_balancer: float = 0.5,
        differential_lock: float = 0.0,
    ) -> None:
        """Wyślij sterowanie do silnika wyścigowego.

        throttle         – gaz [0.0–1.0]
        brake            – hamulec [0.0–1.0]
        steer            – skręt [-1.0 lewo … 1.0 prawo]
        gear_shift       – zmiana biegu (domyślnie NONE)
        brake_balancer   – rozkład hamowania przód/tył (domyślnie 0.5)
        differential_lock – blokada dyferencjału (domyślnie 0.0)
        """
        self._actions.set_controls(
            Controls(
                throttle=throttle,
                brake=brake,
                steering=steer,
                gear_shift=gear_shift,
                brake_balancer=brake_balancer,
                differential_lock=differential_lock,
            )
        )

    def request_back_to_track(self) -> None:
        """Żądaj powrotu pojazdu na tor (np. po wypadnięciu poza nawierzchnię).
        Podlega cooldownowi — sprawdź command_cooldowns.back_to_track_remaining_ms."""
        self._actions.request_back_to_track()

    def request_emergency_pitstop(self) -> None:
        """Żądaj natychmiastowego wjazdu do alei serwisowej.
        Podlega cooldownowi — sprawdź command_cooldowns.emergency_pitstop_remaining_ms."""
        self._actions.request_emergency_pitstop()

    def set_next_pit_tire_type(self, tire_type: TireType) -> None:
        """Ustaw typ opon, które zostaną założone podczas następnego pit stopu.

        tire_type – wybrany TireType (HARD, SOFT lub WET)
        """
        self._actions.set_next_pit_tire_type(tire_type)


class BotProtocol(Protocol):
    """Interfejs, który musi implementować każdy bot.

    on_tick – wywoływana przez środowisko co tick; tutaj bot podejmuje decyzje
              i wywołuje ctx.set_controls(...) oraz inne komendy.
    """
    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None: ...