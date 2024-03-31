function out = deadzone(l,data,h)
if(data<l)
    out=l;
elseif(data>h)
    out=h;
else
    out=data;
end
end

