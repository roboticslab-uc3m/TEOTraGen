
prueba = trajectories.joints.q;
for ii=1:size(prueba,2)
  if prueba(5,ii) < -0.2793,
    prueba(5,ii) = -0.2793;
  end
  if prueba(11,ii) < -0.2793,
    prueba(11,ii) = -0.2793;
  end
end

trajectories.joints.q = prueba;