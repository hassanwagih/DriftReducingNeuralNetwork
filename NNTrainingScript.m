train2 = [seq00T, seq01T, seq02T, seq03T, seq04T, seq05T, seq06T, seq07T, seq08T, seq09T];
label2 = [seq00L, seq01L, seq02L, seq03L, seq04L, seq05L, seq06L, seq07L, seq08L, seq09L];
net17= feedforwardnet(30,'trainbr');
net17.trainParam.min_grad=1e-10;
net17.trainParam.mu=0.0050;
net17.trainParam.epochs=5000;
net17=train(net17,train2,label2,'useParallel','yes');