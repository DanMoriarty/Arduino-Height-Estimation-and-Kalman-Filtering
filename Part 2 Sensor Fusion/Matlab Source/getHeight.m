function [ h ] = getHeight( p, p0 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    d = p/p0;
    e = d^0.19;
    h =44330*(1-e);

end

