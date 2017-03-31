    

    businessCard   = imread('matlabPic2.png');
     ocrResults     = ocr(businessCard)
     recognizedText = ocrResults.Text;
     figure;
     imshow(businessCard);
     text(600, 150, recognizedText, 'BackgroundColor', [1 1 1]);