# Behaviour Tree Design für den Turtlebot 4

## Ziele und Randbedingungen
- Sprachbefehle in Echtzeit in modulare Skills übersetzen und dynamisch zu Missionssequenzen kombinieren
- Reaktives Verhalten: Sicherheitsüberwachung besitzt stets Priorität und kann laufende Aktionen pre-empten
- Gemeinsame Wissensbasis über `Blackboard` und `EventBus`, inklusive Weltkarte und Objektverfolgung
- Integration vorhandener Komponenten (`Map`, Lidar-, Kamera- und TF-Processor, TwistWrapper) ohne deren API zu ändern

## Makrostruktur des Baums
```
Root (Selector, memory=False)
├── SafetySupervisor (ReactiveSequence)
│   ├── HazardDetected? (Condition)
│   ├── EmergencyStop (Action)
│   └── RecoveryManager (Selector)
│       ├── ClearLocalCostmap (Action)
│       ├── BackupAndScan (Sequence)
│       └── RequestAssistance (Action)
├── MissionManager (Parallel, success_on_all=True, failure_on_one=True)
│   ├── PerceptionPipeline (Parallel, success_on_all=True)
│   │   ├── UpdateTargetTracking (Action)
│   │   └── UpdateRobotState (Action)
│   └── TaskExecutor (ReactiveSequence)
│       ├── CommandListener (Condition + Action)
│       ├── TaskDispatcher (Selector)
│       │   ├── ActiveSkillRunning? (Condition)
│       │   └── SkillFactory (BehaviourTree ID decorator)
│       └── PostTaskCleanup (Action)
└── IdleBehaviour (Action)
```

- `Selector` an der Wurzel garantiert, dass zuerst geprüft wird, ob ein Notfall vorliegt; nur wenn der Sicherheitszweig `FAILURE` zurückgibt, darf der Missionszweig arbeiten.
- `ReactiveSequence` (PyTrees: `Sequence` mit `memory=False`) im Sicherheitszweig sorgt dafür, dass nach einem Hazard der Emergency-Stop unmittelbar ausgeführt und danach ein Recovery-Verhalten gewählt wird.
- `MissionManager` läuft parallel aus Wahrnehmungs-Updates und eigentlicher Task-Ausführung; scheitert eine der beiden Komponenten, bricht der gesamte Zweig ab und das Safety-Handling übernimmt.

## Sicherheits- und Recovery-Zweig
- **HazardDetected?**: Liest `BlackboardDataKey.LIDAR_OBSTACLE_PRESENT`, `BlackboardDataKey.TARGET_REACHED`, zusätzliche Flags (z. B. Bumper, Not-Aus). Gibt `SUCCESS`, wenn eine Gefahr erkannt wird.
- **EmergencyStop**: Publiziert einen Null-Twist, cancelt Nav2-Aktionen und setzt Flags `ROBOT_IS_TURNING=False`, `TASK_PAUSED=True` auf dem Blackboard.
- **RecoveryManager**: wählt die zuerst passende Recovery-Strategie.
  - `ClearLocalCostmap`: sendet `clear_entirely_local_costmap` (Nav2 Service) und prüft Ergebnis.
  - `BackupAndScan`: Sequenz aus kurzem Rückwärtsfahren (`TwistWrapper`), Drehen in zufällige Richtung und erneutes Lidar-Check.
  - `RequestAssistance`: Letzter Fallback; veröffentlicht ein Event, dass menschliche Hilfe erforderlich ist (z. B. Lautsprecher-Ausgabe).

## Missionszweig im Detail
### PerceptionPipeline
- Läuft parallel, um kontinuierlich Kamera-, Depth- und Lidar-Daten zu verarbeiten.
- `UpdateTargetTracking`: greift auf `Map` zu und hält `BlackboardDataKey.ROBOT_MAP` aktuell.
- `UpdateRobotState`: aktualisiert Pose, Orientierung und setzt `BlackboardDataKey.ROBOT_IS_TURNING` basierend auf laufenden Bewegungen.

### TaskExecutor
1. **CommandListener**
   - Abonniert natürliche Sprache (z. B. über Speech-To-Text Topic) und gibt `SUCCESS`, sobald neue Befehle im `BlackboardDataKey.COMMAND_QUEUE` liegen.
   - Setzt bei leerer Queue `FAILURE`, wodurch der Parent-Selector den `IdleBehaviour` aktiviert.
2. **TaskDispatcher**
   - `ActiveSkillRunning?`: Prüft `BlackboardDataKey.ACTIVE_SKILL_ID`. Falls ein Skill noch läuft, tickt nur dessen Subtree weiter.
   - `SkillFactory`: wählt anhand des nächsten Kommando-Elements aus der Queue den passenden Skill-Subtree (siehe unten) und hängt ihn via `BehaviourTree.ID`-Decorator unter den Dispatcher.
3. **PostTaskCleanup**
   - Räumt Blackboard-Einträge (`ACTIVE_SKILL_ID`, `TARGET_OBJECT_CLASS`, temporäre Nav-Ziele) auf und schiebt das Kommando in das Verlaufs-Log.

### IdleBehaviour
- Sendet `cmd_vel = 0`, repleniert Energie bei Bedarf, kann eine „Bereit“-Animation auslösen.

## Skill-Bibliothek (modulare Subtrees)
Jeder Skill basiert auf `Sequence(memory=False)` oder `Selector(memory=False)` und hat klar definierte Ein- und Ausgänge über den Blackboard.

### `SearchPersonSkill`
```
Sequence(memory=False)
├── EnsurePerceptionReady (Condition)
├── KnownTargetCoords? (Condition)
├── -> wenn SUCCESS: PublishNavGoal (Action)
│      └── MonitorNavAndTarget (Parallel success_on_all=True)
│          ├── Nav2ActionClient (Action)
│          └── VisionConfirmation (Condition)
└── -> wenn KnownTargetCoords? == FAILURE: RandomExplorationSearch (Selector)
    ├── SpinScan (Action)
    ├── FrontierExploration (Action)
    └── HumanDetection (Condition)
```
- `KnownTargetCoords?` fragt `Map.persistent_tracked_objects`. Bei Treffer: Nav2-Goal (`geometry_msgs/PoseStamped`) wird veröffentlicht.
- Bei Nicht-Wissen aktiviert `RandomExplorationSearch` ein Frontier- bzw. Zufallsexplorationsverhalten. Sobald `HumanDetection` ein Ereignis `TARGET_OBJECT_SELECTED` veröffentlicht, kehrt der Skill zum Nav-Zweig zurück.

### `NavigateToCoordinateSkill`
```
Sequence(memory=False)
├── ValidateGoalFrame (Condition)
├── PublishNavGoal (Action)
├── MonitorProgress (Parallel, success_on_all=True)
│   ├── Nav2ActionClient (Action)
│   └── SafetyHeartbeat (Condition)
└── GoalReached? (Condition)
```
- Nutzt TF-Informationen (`TFSubscriber`) und `TargetReachedDetector`. Bei `FAILURE` des Heartbeats wird der Skill abgebrochen, `TaskDispatcher` versucht eine Recovery oder bricht das Kommando ab.

### `RecognizeObjectSkill`
```
Sequence(memory=False)
├── SetTargetClass (Action)
├── ObjectInFOV? (Condition)
├── AdjustPoseForView (Selector)
│   ├── FineAdjustUsingTF (Action)
│   └── RotateInPlace (Action)
└── ConfirmDetection (Condition)
```
- Unterstützt das Stuhl-Erkennen aus dem Beispiel. `SetTargetClass` setzt `BlackboardDataKey.TARGET_OBJECT_CLASS`. Erkennt das Objekt, publiziert Details auf der Map.

### `RotateSkill`
```
Sequence(memory=False)
├── PrepareTwist (Action)
├── ExecuteRotation (Action)
└── UpdateRobotIsTurning(False) (Action)
```
- Nutzt vorhandene `TurnAround`-Logik, erweitert diese um Parametrisierung (z. B. Winkel, Richtung) basierend auf Kommando.

## Verarbeitung natürlicher Sprache
1. **Speech-To-Text / NLP-Pipeline** erzeugt eine Liste strukturierter Kommandos (`Command` Objekte mit Typ, Parametern, Priorität).
2. **CommandInterpreter** (neuer Node oder im bestehenden `TextToTurtlebotNode`) mappt natürliche Sprache auf Skill-Typen (`search_person`, `navigate_to_pose`, `detect_object`, `rotate`).
3. **TaskQueueManager** schreibt Kommandos in `BlackboardDataKey.COMMAND_QUEUE` (FIFO), erlaubt Priorisierung (z. B. `urgent`-Flag -> front insertion).
4. **SkillFactory** erstellt pro Kommando einen dedizierten Subtree:
   - Parameter werden über Blackboard-Schlüssel (`CURRENT_GOAL_POSE`, `TARGET_OBJECT_CLASS`, `EXPLORATION_MODE`) bereitgestellt.
   - Jeder Skill registriert clean-up Hooks, um den Blackboard-Zustand zurückzusetzen.

## Blackboard-Schnittstellen (Erweiterungen)
- `COMMAND_QUEUE`: Liste offener Kommandos.
- `ACTIVE_SKILL_ID`: aktuell laufender Skill.
- `CURRENT_GOAL_POSE`: Pose für Nav2.
- `TARGET_OBJECT_CLASS`: zu detektierende Klasse (z. B. `person`, `chair`).
- `EXPLORATION_MODE`: bool für Random-Exploration.
- `NAVIGATION_STATUS`: Feedback von Nav2 (`active`, `success`, `aborted`).
- `TASK_PAUSED`: Flag, wenn Safety eine Aktion unterbricht.

## Ticking und Threading
- Bestehender Tick-Thread (`TextToTurtlebotNode._run_tree_loop`) ruft `root.tick_once()` im Intervall `0.1s` auf.
- `MissionManager`-Parallelzweig erlaubt, dass Perzeption weiterläuft, selbst wenn ein Skill blockiert; dennoch sollte jede Action nicht-blockierende Aufrufe nutzen (Nav2-Action-Clients, Event-getriebene Rückmeldungen).
- Recovery-Zweig setzt `TASK_PAUSED=True`, woraufhin `TaskDispatcher` den laufenden Skill beim nächsten Tick pausiert oder abbricht.

## Erweiterungsschritte für die Implementierung
1. **Baumkonstruktion** in `nodes/text_to_turtlebot_node.py` anpassen: Composites und Behaviours erstellen, `setup()`-Methoden der neuen Nodes initialisieren.
2. **Blackboard-Erweiterungen** um zusätzliche Keys und Events vornehmen.
3. **Skill-Bibliothek** als eigenständige Module (z. B. `behaviours/skills/search_person.py`) implementieren; jede Klasse beherbergt `setup`, `initialise`, `update`, `terminate`.
4. **Sprachpipeline**: Parser zwischen Text und `Command`-Objekten bauen (`nodes/text_to_turtlebot_node.py:24` ist guter Einstiegspunkt).
5. **Testing**: Simulation im Gazebo (Nav2) + Replay-Cases für Sprachbefehle, Safety-Trigger durch Lidar-Events.

Mit dieser Struktur bleibt der Turtlebot reaktiv und sicher, kann aber gleichzeitig komplexe, zur Laufzeit definierte Sequenzen von Skills ausführen, die über natürliche Sprache eingespeist werden.
