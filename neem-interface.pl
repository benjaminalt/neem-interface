:- rdf_meta(mem_event_create(r,r,r)).
:- use_module(library('db/mongo/client')).
:- dynamic execution_agent/1.

mem_clear_memory() :-
    drop_graph(user),
    tf_mem_clear, mng_drop(roslog, tf).

mem_episode_start(Action, TaskType, EnvOwl, EnvOwlIndiName, EnvUrdf, AgentOwl, AgentOwlIndiName, AgentUrdf) :-
    get_time(StartTime),
    mem_episode_start(Action, TaskType, EnvOwl, EnvOwlIndiName, EnvUrdf, AgentOwl, AgentOwlIndiName, AgentUrdf, StartTime).

mem_episode_start(Action, TaskType, EnvOwl, EnvOwlIndiName, EnvUrdf, AgentOwl, AgentOwlIndiName, AgentUrdf,
                  StartTime) :-
    retractall(execution_agent(_)),
    tf_logger_disable,
    mem_clear_memory,
    tf_logger_enable,
    % load_owl('package://knowrob/owl/knowrob.owl',[namespace(knowrob)]),   % Is always loaded when knowrob starts
    load_owl(EnvOwl),
    load_owl(AgentOwl),
    urdf_load(AgentOwlIndiName, AgentUrdf, [load_rdf]),
    urdf_load(EnvOwlIndiName, EnvUrdf, [load_rdf]),
    assertz(execution_agent(AgentOwlIndiName)),
    execution_agent(Agent),
    kb_project([
        new_iri(Episode, soma:'Episode'), is_episode(Episode), % Using new_iri here and below is a hideous workaround for a KnowRob bug, see https://github.com/knowrob/knowrob/issues/299
        new_iri(Action, dul:'Action'), is_action(Action),
        new_iri(TimeInterval, dul:'TimeInterval'), holds(Action, dul:'hasTimeInterval', TimeInterval), holds(TimeInterval, soma:'hasIntervalBegin', StartTime),
        new_iri(Task, dul:'Task'), instance_of(Task,TaskType), executes_task(Action,Task),
        is_setting_for(Episode,Action),
        is_performed_by(Action,Agent),
        new_iri(Role, soma:'AgentRole'), has_type(Role, soma:'AgentRole'), has_role(Agent,Role)
    ]),
    % notify_synchronize(event(Action)),
    !.

%is_recording_episode(Result) :- assertz(cramEpisodeName('None')), retract(cramEpisodeName('None')), (cramEpisodeName(Name) -> Result = Name ; Result = 'NoName').
%delete_episode_name(Name) :- retract(cramEpisodeName(Name)).

mem_episode_stop(NeemPath) :-
    get_time(EndTime),
    mem_episode_stop(NeemPath, EndTime).

mem_episode_stop(NeemPath, EndTime) :-
    kb_call([
        is_episode(Episode), is_action(Action), is_setting_for(Episode,Action),
        holds(Action, dul:'hasTimeInterval', TimeInterval)
    ]),
    kb_project([
        holds(TimeInterval, soma:'hasIntervalEnd', EndTime)
    ]),
    get_time(CurrentTime), atom_concat(NeemPath,'/',X1), atom_concat(X1,CurrentTime,X2), memorize(X2), mem_clear_memory.

mem_event_set_failed(Action) :- kb_project(action_failed(Action)).

mem_event_set_succeeded(Action) :- kb_project(action_succeeded(Action)).

mem_event_add_diagnosis(Situation, Diagnosis) :- kb_project(satisfies(Situation, Diagnosis)).

mem_add_subaction_with_task(ParentAction,SubActionType,TaskType,SubAction) :-
    execution_agent(Agent),
    kb_project([
        new_iri(SubAction, SubActionType), is_individual(SubAction), instance_of(SubAction,SubActionType),
        new_iri(Task, TaskType), is_individual(Task), instance_of(Task,TaskType), executes_task(SubAction,Task),
        % % has_subevent(ParentAction,SubAction), <-- has_subevent is currently broken in KnowRob (https://github.com/knowrob/knowrob/issues/300)
        holds(ParentAction,dul:hasConstituent,SubAction), % replacement for has_subevent
        is_performed_by(SubAction,Agent)
    ]),!.

% mem_event_end(Event) :- execution_agent(Agent), get_time(CurrentTime), kb_call([triple(Event,dul:'hasTimeInterval',TimeInterval), triple(TimeInterval,soma:'hasIntervalBegin', Start), executes_task(Event,Task)]),kb_unproject(TimeInterval, soma:'hasIntervalEnd', _),kb_project([holds(TimeInterval, soma:'hasIntervalEnd', CurrentTime),has_type(Role, soma:'AgentRole'), has_role(Agent,Role) during Event,task_role(Task, Role)]),!.

% mem_event_begin(Event) :- get_time(CurrentTime),kb_project(occurs(Event) since CurrentTime),!.

%belief_perceived_at(ObjectType, Frame, Object) :- get_time(CurrentTime),execution_agent(Agent),kb_project([has_type(Object,ObjectType),is_at(Object,Frame) since CurrentTime]).

belief_perceived_at(ObjectType, Mesh, Rotation, Object) :- kb_project([has_type(Object,ObjectType),has_type(ShapeRegion, soma:'MeshShape'), has_type(Shape, soma:'Shape'), triple(Object, soma:'hasShape', Shape), triple(Shape, dul:'hasRegion', ShapeRegion), triple(ShapeRegion, soma:'hasFilePath', Mesh),has_type(Origin,soma:'Origin'),triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin),triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasPositionVector', term([0.0,0.0,0.0])),triple(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#hasOrientationVector',term(Rotation))]).

belief_perceived_at(ObjectType, Object) :- kb_project([has_type(Object,ObjectType)]).

mem_tf_set(Object, Pose, Timestamp) :-
    time_scope(=(Timestamp), =<('Infinity'), FScope),
    tf_set_pose(Object, Pose, FScope).

mem_tf_get(Object, Pose) :-
    current_scope(QScope),
    tf_get_pose(Object, Pose, QScope, _).

mem_tf_get(Object, Pose, Timestamp) :-
    time_scope(=(Timestamp), =(Timestamp), QScope),
    tf_get_pose(Object, Pose, QScope, _).

mem_add_participant_with_role(Action, ObjectId, RoleType) :-
    kb_call(executes_task(Action, Task)),
    kb_project([
        has_participant(Action,ObjectId),
        new_iri(Role, dul:'Role'), is_individual(Role), instance_of(Role, RoleType),
        has_task_role(Task, Role),
        has_role(ObjectId, Role)
    ]).

add_parameter(Task, ParameterType, RegionType, Parameter) :-
    kb_project([
        new_iri(Parameter, dul:Parameter),
        has_type(Parameter, ParameterType),
        has_parameter(Task, Parameter),
        new_iri(Region, dul:Region),
        has_type(Region, RegionType),
        has_assignment(Parameter, Region)
    ]).

add_named_parameter(Task, ParameterType, ParameterName, RegionType, Parameter) :-
    add_parameter(Task, ParameterType, RegionType, Parameter),
    kb_project(holds(Parameter, soma:hasNameString, ParameterName)).

% set_parameter_value_during_interval(Parameter, ParameterValue, Begin, End) :-
%     has_assignment(Parameter, Region),
%     kb_project(holds(Region, dul:hasRegionDataValue, ParameterValue) during [Begin, End]).

set_parameter_value(Parameter, ParameterValue) :-
    has_assignment(Parameter, Region),
    kb_project(holds(Region, dul:hasRegionDataValue, ParameterValue)).

add_grasping_parameter(Action,GraspingOrientationType) :- kb_call(executes_task(Action, Task)), kb_project([has_type(GraspingOrientation,GraspingOrientationType), has_type(GraspingOrientationConcept,'http://www.ease-crc.org/ont/SOMA.owl#GraspingOrientation'), has_parameter(Task,GraspingOrientationConcept),holds(GraspingOrientationConcept, dul:classifies, GraspingOrientation),has_region(Action,GraspingOrientation)]),!.

add_comment(Entity,Comment) :- kb_project(triple(Entity, 'http://www.w3.org/2000/01/rdf-schema#comment', Comment)).
ros_logger_start :- process_create(path('rosrun'),['mongodb_log', 'mongodb_log.py','__name:=topic_logger', '--mongodb-name', 'roslog', '/tf_projection', '/tf'],[process(PID)]),asserta(ros_logger_pid(PID)).
ros_logger_stop :-     ros_logger_pid(PID),
    retractall(ros_logger_pid(PID)),
    process_create(path(rosnode), ['kill', '/topic_logger'],
        [process(KillPID)]),process_wait(KillPID, _),
    process_wait(PID, _),
    process_create(path(rosnode),['cleanup'],
        [stdin(pipe(In)), detached(true), process(TLPID)]),
    writeln(In,'y'),flush_output(In), process_wait(TLPID, _),
    print_message(informational,'Topic Logger stopped').



% test_tf_query :- kb_call([triple('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_DCJBWPIE',dul:'hasTimeInterval',_O), triple(_O, soma:'hasIntervalBegin', _T2)]),time_scope(=<(_T2), >=(_T2), QScope),writeln(_T2),tf_get_pose('base_footprint', ['map',Position,Orientation], QScope, _),!.
