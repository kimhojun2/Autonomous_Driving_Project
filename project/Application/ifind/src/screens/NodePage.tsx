import AsyncStorage from '@react-native-async-storage/async-storage';
import firestore from '@react-native-firebase/firestore';
import React, { useEffect, useRef, useState } from "react";
import { Animated, Dimensions, Image, Modal, PermissionsAndroid, Platform, StyleSheet, Text, TouchableOpacity, TouchableWithoutFeedback, View } from "react-native";
import Geolocation from 'react-native-geolocation-service';
import MapView, { Marker } from 'react-native-maps';
import { SafeAreaProvider } from "react-native-safe-area-context";
import mapdefault from '../styles/mapdefault';
import theme from "../styles/theme";

const { width, height } = Dimensions.get('window');

export const NodePage = () => {
    interface Location {
        id: string;
        latitude: number;
        longitude: number;
        select: string[];
        stop: string;
    }

    const [modalVisible, setModalVisible] = useState(false);
    const [userStop, setUserStop] = useState('');
    const [locations, setLocations] = useState<Location[]>([]);
    const [currentLocation, setCurrentLocation] = useState({
        latitude: mapdefault.latitude,
        longitude: mapdefault.longitude,
        latitudeDelta: mapdefault.latitudeDelta,
        longitudeDelta: mapdefault.longitudeDelta,
    });
    const [currentIndex, setCurrentIndex] = useState(0); // 현재 표시중인 위치의 인덱스
    const slideAnim = useRef(new Animated.Value(0)).current; // 애니메이션 값

    useEffect(() => {
        const fetchUserDataAndLocations = async () => {
            try {
                const uid = await AsyncStorage.getItem('userUID');
                if (!uid) throw new Error('User UID not found');

                const userResponse = await firestore().collection('users').doc(uid).get();
                const userData = userResponse.data();
                if (userData && userData.stop) {
                    setUserStop(userData.stop);
                }

                const response = await firestore().collection('morai').doc('node').get();
                const data = response.data();
                if (data) {
                    const locationsArray = Object.keys(data).map(key => ({
                        id: key,
                        latitude: data[key].latitude,
                        longitude: data[key].longitude,
                        select: data[key].select || [],
                        stop: data[key].stop,
                    }));
                    setLocations(locationsArray);
                    if (locationsArray.length > 0) {
                        const firstLocation = locationsArray[0];
                        setCurrentLocation({
                            latitude: firstLocation.latitude,
                            longitude: firstLocation.longitude,
                            latitudeDelta: mapdefault.latitudeDelta,
                            longitudeDelta: mapdefault.longitudeDelta,
                        });
                    }
                }
            } catch (error) {
                console.error(error);
            }
        };

        fetchUserDataAndLocations();
    }, []);

    useEffect(() => {
        const fetchLocations = async () => {
            const response = await firestore().collection('morai').doc('node').get();
            const data = response.data();
            if (data) {
                // 위치 데이터 배열 생성
                const locationsArray = Object.keys(data).map(key => ({
                    id: key,
                    latitude: data[key].latitude,
                    longitude: data[key].longitude,
                    select: data[key].select || [],
                    stop: data[key].stop,
                }));
    
                // locations 상태 업데이트
                setLocations(locationsArray);
    
                // 0번 위치 데이터를 기준으로 currentLocation 업데이트
                if (locationsArray.length > 0) {
                    const firstLocation = locationsArray[1]; // 첫 번째 위치
                    setCurrentLocation({
                        latitude: firstLocation.latitude,
                        longitude: firstLocation.longitude,
                        latitudeDelta: mapdefault.latitudeDelta,
                        longitudeDelta: mapdefault.longitudeDelta,
                    });
                }
            }
        };
        fetchLocations().then(() => {
            // console.log(locations);
        });
    }, []);

    const selectLocation = async (selectedId: string) => {
        const uid = await AsyncStorage.getItem('userUID');
        if (!uid) return;
    
        // Firestore 업데이트 로직
        locations.forEach(async (location) => {
            const nodeDocRef = firestore().collection('morai').doc('node');
            if (location.id === selectedId && !location.select.includes(uid)) {
                await nodeDocRef.update({
                    [`${selectedId}.select`]: firestore.FieldValue.arrayUnion(uid)
                });
            } else {
                await nodeDocRef.update({
                    [location.id + '.select']: firestore.FieldValue.arrayRemove(uid)
                });
            }
        });
    
        const selectedLocation = locations.find(location => location.id === selectedId);
        if (selectedLocation) {
            const userDocRef = firestore().collection('users').doc(uid);
            await userDocRef.update({
                stop: selectedLocation.stop
            });
    
            // 사용자가 선택한 새로운 위치 정보로 userStop 상태 업데이트
            setUserStop(selectedLocation.stop);
        }
    
        setModalVisible(false);
    };
    
    useEffect(() => {
        const requestLocationPermission = async () => {
            if (Platform.OS === 'android') {
                const granted = await PermissionsAndroid.request(
                    PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
                    {
                        title: "위치 정보 접근 권한",
                        message: "이 앱은 위치 정보 접근 권한이 필요합니다.",
                        buttonNeutral: "나중에 묻기",
                        buttonNegative: "거부",
                        buttonPositive: "허용"
                    }
                );
                return granted === PermissionsAndroid.RESULTS.GRANTED;
            }
            return true;
        };

        const initializeLocation = async () => {
            const hasPermission = await requestLocationPermission();
            if (hasPermission) {
                Geolocation.watchPosition(
                    (position) => {
                        setCurrentLocation({
                            latitude: position.coords.latitude,
                            longitude: position.coords.longitude,
                            latitudeDelta: mapdefault.latitudeDelta,
                            longitudeDelta: mapdefault.longitudeDelta,
                        });
                    },
                    (error) => {
                        console.error(error);
                    },
                    { enableHighAccuracy: true, distanceFilter: 1 }
                );
            } else {
                console.log("위치 정보 접근 권한이 거부되었습니다.");
            }
        };

        initializeLocation();
}, []);

const slideTo = (index: number) => {
    // 현재 인덱스 설정
    setCurrentIndex(index);

    const pageWidth = 290;
    const offsetX = (width - pageWidth);
    const toValue = -(index * width);

    // 애니메이션 실행
    Animated.spring(slideAnim, {
        toValue: toValue,
        useNativeDriver: true,
    }).start();
};
const handlePrev = () => {
    const nextIndex = currentIndex > 0 ? currentIndex - 1 : locations.length - 1;
    setCurrentIndex(nextIndex);
    slideTo(nextIndex);
    updateMapLocation(nextIndex);
};

const handleNext = () => {
    const nextIndex = (currentIndex + 1) % locations.length;
    setCurrentIndex(nextIndex);
    slideTo(nextIndex);
    updateMapLocation(nextIndex);
};

const updateMapLocation = (index:number) => {
    const location = locations[index];
    if (location) {
        setCurrentLocation({
            latitude: location.latitude,
            longitude: location.longitude,
            latitudeDelta: mapdefault.latitudeDelta/2,
            longitudeDelta: mapdefault.longitudeDelta/2,
        });
    }
};


return (
    <SafeAreaProvider>
        <View style={styles.container}>
            <Text style={styles.title}>승하차 위치 변경</Text>
            <View style={styles.separator} />
            
            <TouchableOpacity
                        style={styles.searchBox}
                        onPress={() => setModalVisible(true)}
                    >
                        <Text style = {styles.nowStop}>{userStop ? `현재 탑승지: ${userStop}` : '위치 선택하기'}</Text>
                </TouchableOpacity>

                <Modal animationType="slide" transparent={true} visible={modalVisible} onRequestClose={() => setModalVisible(false)}>
                    <TouchableWithoutFeedback onPress={() => setModalVisible(false)}>
                        <View style={styles.modalOverlay}>
                        <TouchableWithoutFeedback>
                                <View style={styles.modalContent}>
                                    {/* Animated.View를 사용하여 위치 정보 슬라이드 애니메이션 적용 */}
                                    <Animated.View
                                        style={[
                                            styles.scrollViewContainer,
                                            {
                                                flexDirection: 'row',
                                                transform: [{ translateX: slideAnim }], // 애니메이션 적용
                                            },
                                        ]}
                                    >
                                        {locations.map((location, index) => (
                                            <View key={index} style={styles.modalPage}>
                                                <Text style={styles.modalTitle}>{` ${location.stop}`}</Text>
                                                <TouchableOpacity style={styles.changeButton} onPress={() => selectLocation(location.id)}>
                                                    <Text style={styles.changeButtonText}>변경하기</Text>
                                                </TouchableOpacity>
                                            </View>
                                            
                                        ))}
                                    </Animated.View>
                                    <View style={styles.navigationContainer}>
                                        <TouchableOpacity onPress={handlePrev}>
                                            <Text style={styles.navText}>이전</Text>
                                        </TouchableOpacity>
                                        <TouchableOpacity onPress={handleNext}>
                                            <Text style={styles.navText}>다음</Text>
                                        </TouchableOpacity>
                                    </View>
                                </View>
                            </TouchableWithoutFeedback>
                        </View>
                    </TouchableWithoutFeedback>
                </Modal>
    
          {/* 지도 */}
            <MapView
                    style={styles.map}
                    initialRegion={currentLocation}
                    region={currentLocation}
                >
                    {locations.map((location) => (
                        <Marker
                            key={location.id}
                            coordinate={{
                                latitude: location.latitude,
                                longitude: location.longitude,
                            }}
                            title={`${location.stop}`}
                            >
                                <Image
                                    source={require("../asset/stop_icon.png")}
                                    style={styles.nodeicon}
                                    resizeMode="contain"
                                />
                            </Marker>
                        
                    ))}
                </MapView>
            </View>
    </SafeAreaProvider>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        alignItems: 'center',
        backgroundColor: '#fff',
    },
    title:{
        paddingTop: 10,
        margin: 10,
        fontSize: 35,
        fontFamily: theme.mainfont,
        // fontWeight: "bold",
        color: theme.mainDarkGrey,
    },
    map: {
        width: '90%',
        height: '70%',
        backgroundColor: 'grey',
    },
    separator: {
        width: '90%',
        height: 1,
        backgroundColor: theme.navicolor,
        marginBottom: 5,
    },
    searchBox: {
        width: '90%',
        color: theme.mainDarkGrey,
        height: 50,
        padding: 10,
        margin: 25,
        backgroundColor: '#fff',
        alignItems: 'center',
        borderWidth: 1,
        borderColor: theme.lineblue,
        borderRadius: 20,
    },
    nowStop: {
        fontSize: 23,
        fontFamily: theme.mainfont,
        color: theme.mainDarkGrey,
        // fontWeight: 'bold',
    },
    scrollViewContainer: {
        paddingVertical: 20,
    },
    modalOverlay: {
        flex: 1,
        justifyContent: 'flex-end',
        alignItems: 'center',
    },
    modalContent: {
        margin: 40,
        width: '80%',
        maxHeight: '80%',
    },
    modalPage: {
        justifyContent: 'center',
        width: '100%',
        marginLeft: '10%',
        marginRight: '10%',
        padding: 20,
        alignItems: 'center',
        backgroundColor: 'white',
        borderRadius: 10,
        marginHorizontal:20,
        shadowColor: "#000",
        shadowOffset: {
            width: 0,
            height: 2
        },
        shadowOpacity: 0.25,
        shadowRadius: 4,
        elevation: 5,
    },
    modalTitle: {
        color: theme.mainDarkGrey,
        fontSize: 26,
        // fontWeight: 'bold',
        fontFamily: theme.mainfont,
        marginBottom: 8,
    },
    changeButton: {
        marginTop: 20,
        backgroundColor: theme.lineblue,
        padding: 10,
        borderRadius: 5,
    },
    changeButtonText: {
        color: 'white',
        fontSize: 17,
        fontFamily: theme.mainfont,
    },
    navigationContainer: {
        flexDirection: 'row',
        marginLeft: 50,
        marginTop: -15,
        marginBottom: 30,
    },
    navText: {
        color: '#fff',
        paddingBottom: 6,
        paddingTop: 6,
        paddingLeft: 13,
        paddingRight: 13,
        backgroundColor: theme.lineblue,
        borderRadius: 20,
        marginHorizontal: 20,
        fontSize: 24,
        fontFamily: theme.mainfont,
    },
    nodeicon: {
        width: 50,
        height: 50,
    }
});

