function error = InvKinError(thetas,mags,posAnimal)
posRobot = oneLegForwKin(thetas,mags);
posRobot = posRobot(:,1:3);
errors = [0;0;0;0];
weights = [1;1;2;2];

for i=1:4
    errors(i) = weights(i) * norm((posRobot(i,:) - posAnimal(i,:))/mags(i));
end
error = norm(errors); 
end