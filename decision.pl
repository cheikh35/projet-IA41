% Fichier de description des mouvements
% BLEIN JAFFALI JARDINO RUFFY - P2015


%---------INIT PROVISOIRE-----------
%prédicat pour instancier les robots comme sur le screenshot que j'utilise pour me permettre de faire les tests

:- module( decision, [
	init/1,
	move/2
] ).

init(_):-
	nb_setval(finList,[]),
	nb_setval(fifoList,[]),
	nb_setval(visitedList,[]),
	nb_setval(solutionList,[]),
	nb_setval(finList,[]),
	nb_setval(yellowRobot, [0,0]),
	nb_setval(greenRobot, [8,1]),
	nb_setval(redRobot, [11,11]),
	nb_setval(blueRobot, [5,7]).

%--------------------------PREDICATS INTERMEDIAIRES DE DEPLACEMENT------------------------


%Prédicats intermédiares permettant de savoir jusqu'où on peut aller en ligne droite dans une direction

%Si un mur autour on ne peut pas/plus bouger
move_haut([X,Y],[X,Y]):-case(X,Y,1,_,_,_),!.
move_bas([X,Y],[X,Y]):-case(X,Y,_,1,_,_),!.
move_droite([X,Y],[X,Y]):-case(X,Y,_,_,_,1),!.
move_gauche([X,Y],[X,Y]):-case(X,Y,_,_,1,_),!.

%Si robot autour on ne peut pas/plus bouger
move_haut([X,Y], [X,Y]):- Y1 is Y-1, nb_getval(blueRobot, [X,Y1]),!.
move_haut([X,Y], [X,Y]):- Y1 is Y-1, nb_getval(redRobot, [X,Y1]),!.
move_haut([X,Y], [X,Y]):- Y1 is Y-1, nb_getval(greenRobot, [X,Y1]),!.
move_haut([X,Y], [X,Y]):- Y1 is Y-1, nb_getval(yellowRobot, [X,Y1]),!.
move_bas([X,Y], [X,Y]):- Y1 is Y+1, nb_getval(blueRobot, [X,Y1]),!.
move_bas([X,Y], [X,Y]):- Y1 is Y+1, nb_getval(redRobot, [X,Y1]),!.
move_bas([X,Y], [X,Y]):- Y1 is Y+1, nb_getval(greenRobot, [X,Y1]),!.
move_bas([X,Y], [X,Y]):- Y1 is Y+1, nb_getval(yellowRobot, [X,Y1]),!.
move_droite([X,Y], [X,Y]):- X1 is X+1, nb_getval(blueRobot, [X1,Y]),!.
move_droite([X,Y], [X,Y]):- X1 is X+1, nb_getval(redRobot, [X1,Y]),!.
move_droite([X,Y], [X,Y]):- X1 is X+1, nb_getval(greenRobot, [X1,Y]),!.
move_droite([X,Y], [X,Y]):- X1 is X+1, nb_getval(yellowRobot, [X1,Y]),!.
move_gauche([X,Y], [X,Y]):- X1 is X-1, nb_getval(blueRobot, [X1,Y]),!.
move_gauche([X,Y], [X,Y]):- X1 is X-1, nb_getval(redRobot, [X1,Y]),!.
move_gauche([X,Y], [X,Y]):- X1 is X-1, nb_getval(greenRobot, [X1,Y]),!.
move_gauche([X,Y], [X,Y]):- X1 is X-1, nb_getval(yellowRobot, [X1,Y]),!.

%Sinon on lance une récurisivité pour voir jusqu'où on peut aller.
move_haut([X,Y], [A,B]):- Y1 is Y-1, move_haut([X,Y1], [A,B]).
move_bas([X,Y], [A,B]):- Y1 is Y+1, move_bas([X,Y1], [A,B]).
move_droite([X,Y], [A,B]):- X1 is X+1, move_droite([X1,Y], [A,B]).
move_gauche([X,Y], [A,B]):- X1 is X-1, move_gauche([X1,Y], [A,B]).

%Prédicat move_rectiligne qui à partir d'une coordonnée actuelle nous renvoie les coordonnées des points atteignables en se déplaçant selon les règles du jeu (en ligne droite et stop dès qu'obstacle)

move_rectiligne(Actuel,Haut,Bas,Gauche,Droite):-
			move_haut(Actuel,Haut),
			move_bas(Actuel,Bas),
			move_droite(Actuel,Droite),
			move_gauche(Actuel,Gauche).

%---------------------------PREDICAT-------------------------

%Prédicat qui filtre le résultat de move_recitiligne et renvoie une liste de coordonnées atteignables depuis un point de départ
%deplacement(+CoordActuel,-ListePoints)

deplacement(Actuel,L):- deplacement(Actuel,L,1).

deplacement(Actuel,[H|R],1):-move_rectiligne(Actuel,H,_,_,_),H\=Actuel,deplacement(Actuel,R,2),!.
deplacement(Actuel,L,1):-move_rectiligne(Actuel,Actuel,_,_,_),deplacement(Actuel,L,2).

deplacement(Actuel,[B|R],2):-move_rectiligne(Actuel,_,B,_,_),B\=Actuel,deplacement(Actuel,R,3),!.
deplacement(Actuel,L,2):-move_rectiligne(Actuel,_,Actuel,_,_),deplacement(Actuel,L,3).

deplacement(Actuel,[G|R],3):-move_rectiligne(Actuel,_,_,G,_),G\=Actuel,deplacement(Actuel,R,4),!.
deplacement(Actuel,L,3):-move_rectiligne(Actuel,_,_,Actuel,_),deplacement(Actuel,L,4).

deplacement(Actuel,[D],4):-move_rectiligne(Actuel,_,_,_,D),D\=Actuel,!.
deplacement(Actuel,[],4):-move_rectiligne(Actuel,_,_,_,Actuel).

mov(A,B):-deplacement(A,L),member(B,L).

%--------------------------PREDICAT MOVE-------------------------

%Prédicat move du jeu
%move( [0,0,0,0,TargetNb,CoordBleu,CoordVert,CoordJaune,CoordRouge],
%     ListeAction).

move([0,0,0,0,TargetNb,XB,YB,XV,YV,XJ,YJ,XR,YR],Actions):-
	target(TargetPos,TargetNb),
	selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,TargetNb,RobotPos),
	
	go(RobotPos,TargetPos),

        nb_setval(solutionList,[TargetPos]),
        recherchesolutiondepart(TargetPos),
	nb_getval(solutionList,Liste),

	convertir(Liste,TargetPos,NewList),

	nb_getval(numRobot,N),
	finalement(NewList,N,Actions),
	format("~n Actions ~w~n",[Actions]),!.

move(_,[]).
%-----------------------SELECTION DU ROBOT A MOUVOIR ----------------------

%Prédicat selectRobot qui renvoie les coordonnées du robot de la même couleur que la cible/target

%Pour la cible multicolore on impose que le robot bleu y aille
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,0,[XB,YB]):-nb_setval(blueRobot, [0,0]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,0).
<
%Pour les cibles bleues
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,1,[XB,YB]):-nb_setval(blueRobot,[0,0]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,0).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,2,[XB,YB]):-nb_setval(blueRobot,[0,0]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,0).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,3,[XB,YB]):-nb_setval(blueRobot,[0,0]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,0).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,4,[XB,YB]):-nb_setval(blueRobot,[0,0]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,0).

%Pour les cibles vertes
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,5,[XV,YV]):-nb_setval(greenRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,1).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,6,[XV,YV]):-nb_setval(greenRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,1).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,7,[XV,YV]):-nb_setval(greenRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,1).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,8,[XV,YV]):-nb_setval(greenRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,1).

%Pour les cibles jaunes
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,9,[XJ,YJ]):-nb_setval(yellowRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,2).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,10,[XJ,YJ]):-nb_setval(yellowRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,2).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,11,[XJ,YJ]):-nb_setval(yellowRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,2).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,12,[XJ,YJ]):-nb_setval(yellowRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(redRobot,[XR,YR]),nb_setval(numRobot,2).

%Pour les cibles rouges
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,13,[XR,YR]):-nb_setval(redRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(numRobot,3).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,14,[XR,YR]):-nb_setval(redRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(numRobot,3).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,15,[XR,YR]):-nb_setval(redRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(numRobot,3).
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,16,[XR,YR]):-nb_setval(redRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(numRobot,3).


%Pour les cibles inaccessible
selectRobot(XB,YB,XV,YV,XJ,YJ,XR,YR,17,[XR,YR]):-nb_setval(redRobot, [0,0]),nb_setval(blueRobot,[XB,YB]),nb_setval(greenRobot,[XV,YV]),nb_setval(yellowRobot,[XJ,YJ]),nb_setval(numRobot,3).


%--------------PREDICATS DE MANIPULATION -----------


%Prend la liste d'éléments déjà visités, et des éléments accessibles et renvoie un élément accessible qui n'est pas dans la liste déjà visitée

%-----------

filtre([],[]).
filtre([X|R],L):- nb_getval(fifoList,FIFO),deja1(X,FIFO),filtre(R,L).
filtre([X|R],L):- nb_getval(visitedList,VISIT),deja2(X,VISIT),filtre(R,L).
filtre([X|R],[X|R2]):-filtre(R,R2).

%-----------

deja2([X|Y],[[X|Y]|_]).
deja2(X,[Y|R]):-deja2(X,R).

deja1(X,[[[X|Y]|_]|_]).
deja1(X,[[_|[X|Y]]|_]).
deja1(X,[A|B]):-deja1(X,B).

%----------- Ce que j'ai codé apres ta version --------------------------------------------------------------

extraireTeteDeFifo([X,Y]):-nb_getval(fifoList,[[X,Y]|R]),nb_setval(fifoList,R),nb_getval(finList,FIN),append(FIN,[[X,Y]],FIN2),nb_setval(finList,FIN2).

%--------------------------Solution est appelé la première fois avec une liste comportant uniquement l'arrivée en solution

recherchesolutiondepart(TargetPos):-TargetPos = [],nb_getval(solutionList,[T|L2]),nb_setval(solutionList,L2),!.
recherchesolutiondepart(TargetPos):-
	nb_getval(solutionList,Liste2),
	nb_getval(finList,FIN),
	trouver(TargetPos,Y,FIN),
	append([Y],Liste2,Liste3),
	nb_setval(solutionList,Liste3), 
	recherchesolutiondepart(Y).

	
trouver(X,Y,[[X,Y]|_]):-!.
trouver(X,Y,[_|B]):-trouver(X,Y,B).

convertir([X],X,[]).
convertir([A,Z|B],X,[C|D]):-convertisseur(A,Z,C),convertir([Z|B],X,D).

convertisseur([X|Y],[T|U],VAL):- T>X, VAL is 1 .
convertisseur([X|Y],[T|U],VAL):- Y>U, VAL is 2 .
convertisseur([X|Y],[T|U],VAL):- X>T, VAL is 3 .
convertisseur([X|Y],[T|U],VAL):- U>Y, VAL is 4 .

finalement([],N,[]).
finalement([A|R],N,[N,A|S]):-finalement(R,N,S).

%-----------

copyTeteDeFifo([X,Y]):-nb_getval(fifoList,[[X,Y]|R]),nb_getval(finList,FIN),append(FIN,[[X,Y]],FIN2),nb_setval(finList,FIN2).

%-----------

ajouterElementEnQueueDeFifo(X):-
	nb_getval(fifoList,Fifo),
	append(Fifo,[X],NewFifo),
	nb_setval(fifoList,NewFifo).

%-----------

ajouterListeEnQueueDeFifo([],_).

ajouterListeEnQueueDeFifo([X|R],Pere):-
	nb_getval(fifoList,Fifo),
        

	state_record(X, Pere, State),
        

	append(Fifo,[State],NewFifo),
        
	nb_setval(fifoList,NewFifo),
        
        
        
	ajouterListeEnQueueDeFifo(R,Pere),!.

%-----------

ajouterEnVisited(X):-
	nb_getval(visitedList,Visited),
	append(Visited,[X],NewVisited),
	nb_setval(visitedList,NewVisited).

%-----------

add_list_to_queue([], Queue, Queue).
add_list_to_queue([H|T], Queue, New_queue) :-
    add_to_queue(H, Queue, Temp_queue),
    add_list_to_queue(T, Temp_queue, New_queue).

add_to_queue(X,L,NewL):-append(L,[X],NewL).

empty_List(L):-L=nil.

%-----------

ajoutQSoluce(State):-
	nb_getval(solutionList,Soluce),
	append(Soluce,[State],NewSoluce),
	nb_setval(solutionList,NewSoluce).


ajoutTSoluce(State):-
	nb_getval(solutionList,Soluce),
	append([State],Soluce,NewSoluce),
	nb_setval(solutionList,NewSoluce).

%-----------

%---------------------------BFS ALGO ----------------------------

%-----------

state_record(State, Parent, [State, Parent]).

%-----------

go(Racine, Arrivee) :-
    state_record(Racine, [], Init),

	nb_setval(finList,[]),
    nb_setval(fifoList,[]),
    nb_setval(visitedList,[]),
    nb_setval(solutionList,[]),
    nb_getval(fifoList,L),


    ajouterElementEnQueueDeFifo(Init),
nb_getval(fifoList,F),


    path(Arrivee),!.
    

%-----------

path(Arrivee) :- nb_getval(fifoList,[]),
                  write('Graph parcouru, aucune solution'),nl.

%lorsque on l'a trouvé
path(Arrivee) :- 
    
    copyTeteDeFifo(Next_record),
    
    state_record(Arrivee, _, Next_record),
    nb_getval(fifoList,Hey),
    
    nb_setval(dernier,Next_record).

%tant qu'on l'a pas trouvé
path(Arrivee) :-
    
    extraireTeteDeFifo(Next_record),
    
    state_record(State, Pere, Next_record),
    

    ajouterEnVisited(State),


    deplacement(State,Children),
    
    filtre(Children,Children2),
     
    ajouterListeEnQueueDeFifo(Children2,State), 
    
    path(Arrivee).    

  


%-----------CREATION DU CHEMIN ET DE LA LISTE DE MOUVEMENTS----------

deduire_solution(State_record):- 
    state_record(State,[], State_record),!,
    ajoutTSoluce(State),
   
nb_getval(solutionList,FR),!.


deduire_solution(State_record) :-
    state_record(State, Parent, State_record),
    state_record(Parent, _, Parent_record),
    
    nb_getval(visitedList,Visited),
    member(Parent, Visited),
    deduire_solution(Parent_record),
    ajoutQSoluce(State).

%-----------


% Fichier de description du plateau
% BLEIN JAFFALI JARDINO RUFFY - P2015

% Les coordonnées de chaque case sur le plateau sont des coordonnées de type (x,y).
% Avec x l'abscisse, y l'ordonnée.
% L'origine du repère se trouve tout en bas à gauche du plateau. Un pas vers la droite
% incrémente les abscisses. Un pas vers le haut incrémente les ordonnées.

% le prédicat case/6 nous informe pour chaque case les obstacles autour.

% case (Coord X, Coord Y, Mur en Haut ?, Mur en Bas ?, Mur à Droite ?, Mur à Gauche ?).
% 
% s'il y a un mur en haut par exemple, le champ passe à 1, et il est à 0 sinon. 


%line 1 ok
case(1,1,1,0,1,0).
case(2,1,1,0,0,0).
case(3,1,1,0,0,0).
case(4,1,1,0,0,1).
case(5,1,1,0,1,0).
case(6,1,1,0,0,0).
case(7,1,1,0,0,0).
case(8,1,1,0,0,0).
case(9,1,1,0,0,0).
case(10,1,1,1,0,0).
case(11,1,1,0,0,1).
case(12,1,1,0,1,0).
case(13,1,1,0,0,0).
case(14,1,1,0,0,0).
case(15,1,1,0,0,0).
case(16,1,1,0,0,1).

%line 2 ok
case(1,2,0,0,1,0).
case(2,2,0,0,0,0).
case(3,2,0,0,0,0).
case(4,2,0,0,0,0).
case(5,2,0,0,0,0).
case(6,2,0,0,0,1).
case(7,2,0,1,1,0).
case(8,2,0,0,0,0).
case(9,2,0,0,0,0).
case(10,2,1,0,0,1).
case(11,2,0,0,1,0).
case(12,2,0,0,0,0).
case(13,2,0,0,0,0).
case(14,2,0,0,0,0).
case(15,2,0,0,0,0).
case(16,2,0,0,0,1).

%line 3 ok
case(1,3,0,0,1,0).
case(2,3,0,1,0,0).
case(3,3,0,0,0,0).
case(4,3,0,0,0,0).
case(5,3,0,0,0,0).
case(6,3,0,0,0,0).
case(7,3,1,0,0,0).
case(8,3,0,0,0,0).
case(9,3,0,0,0,0).
case(10,3,0,0,0,0).
case(11,3,0,0,0,0).
case(12,3,0,1,0,1).
case(13,3,0,0,1,0).
case(14,3,0,0,0,0).
case(15,3,0,0,0,0).
case(16,3,0,0,0,1).

%line 4 ok
case(1,4,0,1,1,0).
case(2,4,1,0,0,1).
case(3,4,0,0,1,0).
case(4,4,0,0,0,0).
case(5,4,0,0,0,0).
case(6,4,0,1,0,0).
case(7,4,0,0,0,0).
case(8,4,0,0,0,0).
case(9,4,0,0,0,0).
case(10,4,0,0,0,0).
case(11,4,0,0,0,0).
case(12,4,1,0,0,0).
case(13,4,0,0,0,0).
case(14,4,0,0,0,0).
case(15,4,0,0,0,0).
case(16,4,0,1,0,1).

%line 5 ok
case(1,5,1,0,1,0).
case(2,5,0,0,0,0).
case(3,5,0,0,0,0).
case(4,5,0,0,0,0).
case(5,5,0,0,0,1).
case(6,5,1,0,1,0).
case(7,5,0,0,0,0).
case(8,5,0,0,0,0).
case(9,5,0,0,0,0).
case(10,5,0,0,0,0).
case(11,5,0,0,0,0).
case(12,5,0,0,0,0).
case(13,5,0,0,0,0).
case(14,5,0,0,0,0).
case(15,5,0,0,0,0).
case(16,5,1,0,0,1).

%line 6 ok
case(1,6,0,0,1,0).
case(2,6,0,0,0,0).
case(3,6,0,1,0,1).
case(4,6,0,0,1,0).
case(5,6,0,0,0,0).
case(6,6,0,0,0,0).
case(7,6,0,0,0,0).
case(8,6,0,1,0,1).
case(9,6,0,0,1,0).
case(10,6,0,0,0,0).
case(11,6,0,0,0,0).
case(12,6,0,0,0,0).
case(13,6,0,0,0,1).
case(14,6,0,1,1,0).
case(15,6,0,0,0,0).
case(16,6,0,0,0,1).

%line 7 ok
case(1,7,0,1,1,0).
case(2,7,0,0,0,0).
case(3,7,1,0,0,0).
case(4,7,0,0,0,0).
case(5,7,0,0,0,0).
case(6,7,0,0,0,0).
case(7,7,0,0,0,0).
case(8,7,1,1,0,0).
case(9,7,0,1,0,0).
case(10,7,0,0,0,0).
case(11,7,0,1,0,0).
case(12,7,0,0,0,0).
case(13,7,0,0,0,0).
case(14,7,1,0,0,0).
case(15,7,0,0,0,0).
case(16,7,0,0,0,1).

%line 8 ok
case(1,8,1,0,1,0).
case(2,8,0,0,0,0).
case(3,8,0,0,0,0).
case(4,8,0,0,0,0).
case(5,8,0,0,0,0).
case(6,8,0,0,0,0).
case(7,8,0,0,0,1).
case(8,8,1,1,1,1).
case(9,8,1,1,1,1).
case(10,8,0,0,1,1).
case(11,8,1,0,1,0).
case(12,8,0,0,0,0).
case(13,8,0,0,0,0).
case(14,8,0,0,0,0).
case(15,8,0,0,0,0).
case(16,8,0,0,0,1).

%line 9 ok
case(1,9,0,0,1,0).
case(2,9,0,0,0,0).
case(3,9,0,0,0,0).
case(4,9,0,0,0,0).
case(5,9,0,0,0,0).
case(6,9,0,0,0,0).
case(7,9,0,0,0,1).
case(8,9,1,1,1,1).
case(9,9,1,1,1,1).
case(10,9,0,0,1,0).
case(11,9,0,0,0,0).
case(12,9,0,0,0,0).
case(13,9,0,1,0,0).
case(14,9,0,0,0,0).
case(15,9,0,0,0,0).
case(16,9,0,0,0,1).

%line 10 ok
case(1,10,0,0,1,0).
case(2,10,0,1,0,0).
case(3,10,0,0,0,0).
case(4,10,0,0,0,1).
case(5,10,0,1,1,0).
case(6,10,0,0,0,0).
case(7,10,0,0,0,0).
case(8,10,1,0,0,0).
case(9,10,1,0,0,0).
case(10,10,0,0,0,0).
case(11,10,0,0,0,0).
case(12,10,0,0,0,0).
case(13,10,1,0,0,1).
case(14,10,0,0,1,0).
case(15,10,0,0,0,0).
case(16,10,0,1,0,1).

%line 11 ok
case(1,11,0,0,1,0).
case(2,11,1,0,0,1).
case(3,11,0,0,1,0).
case(4,11,0,0,0,0).
case(5,11,1,0,0,0).
case(6,11,0,0,0,0).
case(7,11,0,0,0,0).
case(8,11,0,0,0,0).
case(9,11,0,0,0,1).
case(10,11,0,1,1,0).
case(11,11,0,0,0,0).
case(12,11,0,0,0,0).
case(13,11,0,0,0,0).
case(14,11,0,0,0,0).
case(15,11,0,0,0,0).
case(16,11,0,1,0,1).

%line 12 ok
case(1,12,0,0,1,0).
case(2,12,0,0,0,0).
case(3,12,0,0,0,0).
case(4,12,0,0,0,0).
case(5,12,0,0,0,0).
case(6,12,0,0,0,0).
case(7,12,0,0,0,0).
case(8,12,0,0,0,0).
case(9,12,0,0,0,0).
case(10,12,1,0,0,0).
case(11,12,0,0,0,0).
case(12,12,0,0,0,0).
case(13,12,0,0,0,0).
case(14,12,0,0,0,0).
case(15,12,0,0,0,0).
case(16,12,0,0,0,1).

%line 13 ok
case(1,13,0,0,1,0).
case(2,13,0,0,0,0).
case(3,13,0,0,0,0).
case(4,13,0,0,0,0).
case(5,13,0,0,0,0).
case(6,13,0,0,0,0).
case(7,13,0,1,0,0).
case(8,13,0,0,0,0).
case(9,13,0,0,0,0).
case(10,13,0,0,0,0).
case(11,13,0,0,0,0).
case(12,13,0,0,0,0).
case(13,13,0,0,0,0).
case(14,13,0,0,0,0).
case(15,13,0,0,0,0).
case(16,13,0,0,0,1).

%line 14 ok
case(1,14,0,0,1,0).
case(2,14,0,0,0,0).
case(3,14,0,0,0,0).
case(4,14,0,0,0,0).
case(5,14,0,0,0,0).
case(6,14,0,0,0,1).
case(7,14,1,0,1,0).
case(8,14,0,0,0,0).
case(9,14,0,0,0,0).
case(10,14,0,1,0,0).
case(11,14,0,0,0,0).
case(12,14,0,0,0,0).
case(13,14,0,0,0,0).
case(14,14,0,0,0,0).
case(15,14,0,1,0,1).
case(16,14,0,0,1,1).

%line 15 ok
case(1,15,0,0,1,0).
case(2,15,0,0,0,0).
case(3,15,0,1,0,1).
case(4,15,0,0,1,0).
case(5,15,0,0,0,0).
case(6,15,0,0,0,0).
case(7,15,0,0,0,0).
case(8,15,0,0,0,0).
case(9,15,0,0,0,1).
case(10,15,1,0,1,0).
case(11,15,0,0,0,0).
case(12,15,0,0,0,0).
case(13,15,0,0,0,0).
case(14,15,0,0,0,0).
case(15,15,1,0,0,0).
case(16,15,0,0,0,1).

%line 16 ok
case(1,16,0,1,1,0).
case(2,16,0,1,0,0).
case(3,16,1,1,0,0).
case(4,16,0,1,0,1).
case(5,16,0,1,1,0).
case(6,16,0,1,0,1).
case(7,16,0,1,0,0).
case(8,16,0,1,0,0).
case(9,16,0,1,0,0).
case(10,16,0,1,0,0).
case(11,16,0,1,0,1).
case(12,16,0,1,1,0).
case(13,16,0,1,0,0).
case(14,16,0,1,0,0).
case(15,16,0,1,0,0).
case(16,16,0,1,0,1).


%Target([+X,+Y],+Id)
%multicouleur
target([8,6],0).
%METTRE DES CROCHETS !
%bleu
target([7,2],1).
target([10,6],2).
target([14,6],3).
target([7,14],4).

%vert
target([12,3],5).
target([6,5],6).
target([2,11],7).
target([15,14],8).

%jaune
target([5,10],9).
target([10,2],10).
target([10,12],11).
target([2,4],12).

%rouge
target([12,10],13).
target([3,15],14).
target([3,6],15).
target([11,8],16).

%inaccessible
target([8,8],17).



%-------------- END ---------------