%% Problem 1
% For an unfair coin with p(H) = 0.8999, determine the correct order of
% queries that should be used to determine the outcome of four coin flips.
% Remember that there are 16 total combinations, so there are 16 total
% possible states.
% TURN IN: turn in the graph that represents the order of the queries and
% the expected number of questions required to determine the state.

clear;
close all;
clc;

%% Possible states
% X = [
%  HHHH (1 way to get 4 heads)
%  THHH (or HTHHH, HHTH, HHHT) (4 ways to get Hx3, Tx1)
%  HHTT (or HTHT, HTTH, THTH, TTHH, THHT) 
% (6 ways to get Hx2, Tx2)
%  HTTT (or THTT, TTHT, TTTH) (4 ways to get Hx1, Tx3)
%  TTTT (1 way to get 4 tails)
%  ]
%
% 1 + 4 + 6 + 4 + 1 = 16 possible states
%% Probability of getting heads
pH = 0.8999;

%% Likelihood of each outcome
pX = [
pH^4;
(pH^3)*(1-pH);
(pH^2)*(1-pH)^2;
pH*((1-pH)^3);
(1-pH)^4;
];

%% Entropy of each state
ent = -(pX(1)*log2(pX(1)) + 4*pX(2)*log2(pX(2)) + 6*pX(3)*log2(pX(3)) ...
    + 4*pX(4)*log2(pX(4)) + pX(5)*log2(pX(5)));

%% Expected number of questions
EQ = ent

%% Question order: descending likelihood
figure
nodes = zeros(1,34);
nodes(1:9) = [0 1 1 3 3 5 5 7 7];

nodes(10:16) = [4 10 10 11 11 12 12];
nodes(17:27) = [6 17 17 18 18 19 19 21 21 23 23];
nodes(28:34) = [8 28 28 29 29 30 30];

treeplot(nodes);
[x,y] = treelayout(nodes);
title(['Order of queries Q; expect ',num2str(EQ),' queries.'])

text(x(1),y(1),"   4 H?")
text(x(2),y(2),"   HHHH")
text(x(3),y(3),"   3 H?")
text(x(4),y(4),"   3H 1T")
text(x(5),y(5),"   2 H?")
text(x(6),y(6),"   2H 2T")
text(x(7),y(7),"   1 H?")
text(x(8),y(8),"   1H 3T")
text(x(9),y(9),"   TTTT")

text(x(10),y(10),"   ''HHH''?")
text(x(11),y(11),"   H first?")
text(x(12),y(12),"   HH first?")
text(x(13),y(13),"   HHTH")
text(x(14),y(14),"   HTHH")
text(x(15),y(15),"   HHTH")
text(x(16),y(16),"   HTHH")

text(x(17),y(17),"   ''HH''?")
text(x(18),y(18),"   HH first?")
text(x(19),y(19),"   ''TT''?")
text(x(20),y(20),"   HHTT")
text(x(21),y(21),"   HH middle?")
text(x(24),y(24),"   THHT")
text(x(25),y(25),"   TTHH")
text(x(22),y(22),"   HTTH")
text(x(23),y(23),"   H first?")
text(x(26),y(26),"   HTHT")
text(x(27),y(27),"   THTH")

text(x(28),y(28),"   ''TTT''?")
text(x(29),y(29),"   H first?")
text(x(30),y(30),"   TT first?")
text(x(31),y(31),"   HTTT")
text(x(32),y(32),"   TTTH")
text(x(33),y(33),"   TTHT")
text(x(34),y(34),"   THTT")