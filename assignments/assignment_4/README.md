# Photogrammetry and Gaussian Splatting

Notaiton

N15 ---> original dataset

N25 ---> original + 10 novel images

A code was written to compare image renders from the same position to get SNIR and PNIR, but since ground truth model was not available to compare with the N15 and N25 reconstructions, its quite useless to deply now. Hence Cloud Compare was used to get some metrics that prove
that the N25 model is densser and has been reconstructed properly even when the new images were taken 

My subjective opinion is given at the end of the document

Two analysis in Cloud Compare

### Gaussian (Normal) Distribution
- Symmetrical bell curve
- Describes random errors with most values near the mean
- Used to model natural noise or measurement error

### Weibull Distribution
- Skewed, asymmetric curve
- Common in reliability engineering, failure analysis, and error modeling
- Flexible shape: can model skewed error distributions where Gaussian fails

Why two ? If the Gaussian fit performs poorly while the Weibull distribution fits well, it indicates that the error distribution is skewed rather than symmetrical.

## Workflow (with softwares used)

1. Load images to Metashape
2. Reconstruct
3. Perform Gaussian Splatting using PostShot (explanation why after)
4. Clean up the splats using Splatstudio and Postshot
5. Take 10 positions different than the camera position around the model and take images
6. Add those 10 images and repeat step 1 to 5.
7. Upload the .ply's to CloudCompare
8. Align the clouds
9. Statistics of that will be uploaded as the document progresses

### Read if you would like to know why the original repo wasn't used
I couldn't setup the Gaussian Splatting on windows because in Visual Studio 2022 the desktop environments with C++ comes with MSVC version not compatible with the repo. For a compatible MSVC I have to go to 2019 and its locked behind a subscription for some reason.
I have ubuntu 24 on my PC and hence that was ruled out. Tried to run a WSL inside windows with 22 to good success, I thought my GPU (RTX 400Ti Super, 16GBVRAM) drivers need to be installed from source, but apparently they are taken from Windows and I broke it by adding the
driver source and updating the package list. Then a new one had version issues, hence these issues carried over to Nerf studio, I had to use a different software called PostShot which turned out to be pretty handy and were able to get really good results


## Initial Metashape reconstruction 

Photo Alignment
- Key Point Limit: 60,000–100,000 which captures more features per image for better matching.
- Tie Point Limit: 40,000–100,000 retains more matched points, improving model precision.

Mesh Reconstruction
- Face Count: 5,000,000–10,000,000+ or just High would suffice

Texture Generation
- Texture Size 16384 to get better details. I went 4x higher but it was just diminishing returns

### Initial Reconstruction of the 15 Images using Metashape

![meta1](https://github.com/user-attachments/assets/f9586926-465d-4e7d-bd30-5231c2648b7c)
![meta2](https://github.com/user-attachments/assets/4289269d-c847-4c21-8da7-b8c0fae9538e)

### Initial Reconstruction of the 15 Images using Gaussian Splats

![before1](https://github.com/user-attachments/assets/9384c58a-7830-4750-ae8e-7ae6b63a647a)
![before2](https://github.com/user-attachments/assets/1286bb82-c7a3-4c49-bd82-2f148607b560)


## Novel Views
The camera was moved around, in all directions, the view was rotated and rendered in the Splat model to take 10 new images of the reconstructed model to add to the original image folder

The new camera positions are N1 to N10
![new1](https://github.com/user-attachments/assets/d49a8551-d354-48fe-b0ce-5b12c74a27e1)

## Reconstruction of N = 25 with Metashape

![final1](https://github.com/user-attachments/assets/d026c1f2-73a4-4a12-8f0b-0e78cc698adc)
![final2](https://github.com/user-attachments/assets/071c2367-c876-40b3-844d-74c829319a81)

## Reconstruction of N = 25 with Gaussian Splats
![after1](https://github.com/user-attachments/assets/aa61baeb-d075-4a92-b47f-353f35a03a41)
![after2](https://github.com/user-attachments/assets/0b04f366-2c79-4534-ae71-37288b97932f)


## Comparison 
The point clouds were uploaded to Cloud Compare to analyse both with respect to each other. These are some of the pictures and results of the analysis

### CloudCompare Distribution Fitting Summary

- **Theoretical Overlap:** 100%  
  Indicates that the two point clouds cover the same spatial area — perfect registration.

-  **Number of Valid Points:** 1,890,182 (100%)  
  Every point in the cloud was successfully compared.

---

#### Gaussian (Normal) Distribution Fit #1
- **Mean:** -6.43  
  → On average, the points in one cloud are 6.43 units behind the other.
- **Standard Deviation:** 1.13  
  → Most differences fall within ±1.13 units around the mean.
- **Chi² Distance:** 659,871  
  → A statistical measure of how well the data fits a Gaussian. Lower is better.

---

#### Gaussian Fit #2 (possibly full cloud-to-cloud comparison)
- **Mean:** -7.59  
- **Standard Deviation:** 3.10  
- **Chi² Distance:** 2,961,190  
  → A worse fit compared to the first, indicating less Gaussian-like error distribution.

---

- **RMS (Root Mean Square Error):** 6.53  
  Average geometric error magnitude across all points.

---


![gauss1](https://github.com/user-attachments/assets/ff548b4c-03ba-409c-b397-12cf86c0400c)


---
#### Weibull Distribution Fit
- **Mode:** -6.21 (most common offset)
- **Skewness:** -0.276  
  → Slight left-skewed error distribution
- **Parameters:**
  - **a (shape):** 5.89
  - **b (scale):** 14.91
  - **Shift:** -11.94
- **Chi² Distance:** 748,268  
  → Better fit than Gaussian in this case
---

![weibull2](https://github.com/user-attachments/assets/f87270d2-52dd-4d6d-8acd-6dbe7b9b94fb)

---



## Subjective optinion

The initial reconstructions came out really well, especially the metashape reconstruction with the higher tie and face counts. While it took more training time to achieve the same results in Gaussian Splatting, I did feel the Splat was more accurate if viewed from a radius around a camera position (not novel). 

While the splat cleanup took as long as the training, perhaps even more I did think the splat came out on top. 

### Did anything improve after novel view synthesis ?

Yes. The local features were more pronounced in both splat and the metahshape model. Especially in the methashape model the places that were not rendered properly were rendered better than the previous iteration of N15. The edges seemed to get worse in both models, however the mentioned radius of viewing angles around an origianl camera position
becamse a bit bigger.

There were less stray splats around the ROI, but 4x more to cleanup around the ROI

Overall I would say N25 was subjectively better than N15, in both metashape and gaussian splatting.








