/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
    keyfrmsLst.push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);
    for (list<KeyFrame *>::iterator lit = keyfrmsLst.begin(), lend = keyfrmsLst.end(); lit != lend; lit++)
    {
        if (pKF == *lit)
        {
            keyfrmsLst.erase(lit);
            break;
        }
    }
    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    keyfrmsLst.clear();
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

bool KeyFrameDatabase::label_check(const KeyFrame* kf, const Frame *F) {
    if (F->labels_(0) != VALID_OBJ){
        // Frame have no label
        // cout << "F->labels_(0) != VALID_OBJ !!" << F->labels_(0) << endl;
        return true;
    }
    if (kf->labels_(0) != VALID_OBJ){
        // cout << "kf->labels_(0) != VALID_OBJ !!" << kf->labels_(0) << endl;
        return false;
    }
    Eigen::Array<float,MAX_OBJECT_NUM,1> result = F->labels_ - kf->labels_;
    result = result.abs();
    float thres = F->labels_.sum() - VALID_OBJ;
    if ((int)result.sum() < (thres+1)){
        cout << F->labels_ << "    " << kf->labels_ << endl;
        return true;
    }
    return false;

}

int KeyFrameDatabase::label_score(const KeyFrame* kf, const Frame *F) {
    if (kf->labels_(0) != VALID_OBJ){
        // cout << "kf->labels_(0) != VALID_OBJ !!" << kf->labels_(0) << endl;
        return 999;
    }
    // for (int i = 0;i<F->labels_.size();i++) {
    //     cout << "frame " << F->labels_(i) << endl;
    //     cout << "key " << kf->labels_(i) << endl;
    // }
    Eigen::Array<float,MAX_OBJECT_NUM,1> result = F->labels_ - kf->labels_;
    result = result.abs();
    return (int)result.sum();

}

bool cmp(pair<KeyFrame*, int>& a,
		pair<KeyFrame*, int>& b)
{
	return a.second < b.second;
}

#include <chrono>
using namespace std;
using namespace std::chrono;
// void KeyFrameDatabase::filter_by_label(list<KeyFrame*>& lkf, const Frame *F) {
void KeyFrameDatabase::filter_by_label(vector<KeyFrame*>& lkf, const Frame *F) {
    vector<pair<KeyFrame*, int>> scores;
    int min_score = 100; // min. is the best
    for(auto kf:keyfrmsLst){
        int score = label_score(kf, F);
        scores.push_back(make_pair(kf,score));
        // cout << "score " << score << endl;
        if (score < min_score) {
            min_score = score;
        }
    }
    // int rank = int(0.2*(lkf.size()+1));
    float thres = (min_score+1)*110/100;
    // sort(scores.begin(), scores.end(), cmp);
    // cout << "min " <<  min_score << "thres " << thres << endl;
    // cout << "max_score " << max_score << "rank " << rank << endl;
    lkf.clear();
    int i = 0;
    for(auto pair:scores){
        // if (i < rank) {
        //     // cout << "pair.second " << pair.second << endl;
        //     lkf.push_back(pair.first);
        // } else {
        //     break;
        // }
        // i++;
        if(pair.second<=thres){
            lkf.push_back(pair.first);
        }
    }
}

double filter_dura=0;
double after=0;
double after1=0;
double bow=0;
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    list<KeyFrame *> lKFsSharingWords;
    // cout << "DetectRelocalizationCandidates !! " << keyfrmsLst.size() << endl;
    // Search all keyframes that share a word with current frame
    {
        auto bow_start = high_resolution_clock::now();
        unique_lock<mutex> lock(mMutex);

        // cout << "BOW: " << Dduration.count() << " microseconds" << endl;
        cout << "Before lKFsSharingWords size" << lKFsSharingWords.size() << endl;
        cout << "# Object " << F->labels_.sum() - VALID_OBJ << endl;

        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->mnRelocQuery != F->mnId && label_check(pKFi, F))
                // if (pKFi->mnRelocQuery != F->mnId)
                {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                else if (pKFi->mnRelocQuery == F->mnId)
                {
                    pKFi->mnRelocWords++;
                }
            }
        }
        auto bow_stop = high_resolution_clock::now();
        auto bow_du = duration_cast<microseconds>(bow_stop - bow_start);
        bow = bow_du.count();
    }

    auto Dstart = high_resolution_clock::now();
    cout << "After lKFsSharingWords size" << lKFsSharingWords.size() << endl;
    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnRelocWords > maxCommonWords)
            maxCommonWords = (*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;

        // auto Dstart = high_resolution_clock::now();
        if (pKFi->mnRelocWords > minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
            cout << "Cal" << endl;
        }
        // auto Dstop = high_resolution_clock::now();
        // auto Dduration = duration_cast<microseconds>(Dstop - Dstart);
        // if (Dduration.count() > 1){
        //     cout << "ID " << pKFi->mnId << " time " << Dduration.count() << endl;
        // }
    }

    auto Dstop = high_resolution_clock::now();
    auto Dduration = duration_cast<microseconds>(Dstop - Dstart);
    after = Dduration.count();
    Dstart = high_resolution_clock::now();

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame *pKFi = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame *pBestKF = pKFi;
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->mnRelocQuery != F->mnId)
                continue;

            accScore += pKF2->mRelocScore;
            if (pKF2->mRelocScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *> spAlreadyAddedKF;
    vector<KeyFrame *> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        const float &si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
    Dstop = high_resolution_clock::now();
    Dduration = duration_cast<microseconds>(Dstop - Dstart);
    after1 = Dduration.count();
    return vpRelocCandidates;
}

// map serialization addition
template<class Archive>
void KeyFrameDatabase::serialize(Archive &ar, const unsigned int version)
{
    // don't save associated vocabulary, KFDB restore by created explicitly from a new ORBvocabulary instance
    // inverted file
    {
        unique_lock<mutex> lock_InvertedFile(mMutex);
        ar & mvInvertedFile;
        ar & keyfrmsLst;
    }
    // don't save mutex
}
template void KeyFrameDatabase::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void KeyFrameDatabase::serialize(boost::archive::binary_oarchive&, const unsigned int);


} //namespace ORB_SLAM
